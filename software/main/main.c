#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

// ============================================================
// ESP32-C6 ODOMETRY SENSOR BOARD  (15-odomBoard)
// ------------------------------------------------------------
// Inputs:
//   - TCA9548A 8-channel I2C mux
//   - AS5600 VERTICAL   encoder on mux channel 6  -> DENC1
//   - AS5600 HORIZONTAL encoder on mux channel 1  -> DENC2
//   - 4x VL53L0X time-of-flight distance sensors:
//       TOF1 = FRONT = channel 2
//       TOF2 = RIGHT = channel 7
//       TOF3 = LEFT  = channel 5
//       TOF4 = LBACK = channel 3  (left-back)
//   - No gyro on this board.
//
// Output over UART1 -> TTL side of RS485 module:
//   DENC1=<delta ticks>,DENC2=<delta ticks>,H=<heading deg>,
//   TOF1=<in>,TOF2=<in>,TOF3=<in>,TOF4=<in>
//
// Notes:
//   - DENC1 = vertical encoder, DENC2 = horizontal encoder. Both are
//     incremental deltas since the last UART packet, not absolute ticks.
//   - H (heading) is always 0.0 -- this board has no gyro, but the field is
//     kept so the packet matches the other boards' format.
//   - TOF1..TOF4 are VL53L0X distances in inches, after a fixed per-sensor
//     offset (see TOF_OFFSET_IN). TOF1=front, TOF2=right, TOF3=left,
//     TOF4=left-back. Out-of-range / no-target reads come back near 321 in.
// ============================================================

// =====================
// PIN SETTINGS
// =====================

// I2C mux bus
#define I2C_SDA_PIN         GPIO_NUM_6
#define I2C_SCL_PIN         GPIO_NUM_7

// UART to TTL/RS485 module
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16

// =====================
// I2C DEVICE SETTINGS
// =====================

#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_FREQ_HZ         400000

#define TCA9548A_ADDR       0x70

#define AS5600_ADDR         0x36
#define AS5600_REG_RAW      0x0E
#define AS5600_TICKS_PER_REV 4096

// All four VL53L0X TOF sensors share this address; the mux selects which one.
#define VL53L0X_ADDR        0x29

// Encoder mux channels.
//   Vertical encoder   -> DENC1 (first field)
//   Horizontal encoder -> DENC2 (second field)
#define ENC1_CHANNEL        6   // vertical
#define ENC2_CHANNEL        1   // horizontal

// Per-encoder direction. Flip to -1 if a tracker's sign is backwards for
// the V5 brain's coordinate convention.
#define ENC1_SIGN           (-1)
#define ENC2_SIGN           (-1)

// VL53L0X TOF sensors by physical position -> packet fields in this order:
//   FRONT, RIGHT, LEFT, LBACK
#define TOF_FRONT_CH        2
#define TOF_RIGHT_CH        7
#define TOF_LEFT_CH         5
#define TOF_LBACK_CH        3

// TOF output is reported in inches. 1 inch = 25.4 mm.
#define MM_PER_INCH         25.4f

// Per-sensor fixed offset (inches) subtracted from each TOF reading to correct
// a constant optical-window setback. Order matches FRONT, RIGHT, LEFT, LBACK.
// Defaulted to the ~1.1" base; recalibrate per sensor against known distances.
static const float TOF_OFFSET_IN[4] = { 1.1f, 1.1f, 1.1f, 1.1f };

// =====================
// UART SETTINGS
// =====================

#define UART_PORT_NUM       UART_NUM_1
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       2048

// =====================
// LOOP TIMING
// =====================

// Main sensor loop: 200 Hz. UART output: 50 Hz.
#define SENSOR_LOOP_HZ      200
#define SENSOR_LOOP_MS      (1000 / SENSOR_LOOP_HZ)
#define UART_OUTPUT_HZ      50
#define OUTPUT_DECIMATION   (SENSOR_LOOP_HZ / UART_OUTPUT_HZ)

static const char *TAG = "ESP32C6_ODOM_RAW";

// =====================
// GLOBAL HANDLES
// =====================

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t tca_handle;
static i2c_master_dev_handle_t as5600_handle;
static i2c_master_dev_handle_t vl53_handle;
static SemaphoreHandle_t i2c_mutex;

// =====================
// SMALL UTILS
// =====================

static int32_t wrap_encoder_delta(uint16_t now_raw, uint16_t last_raw)
{
    int32_t delta = (int32_t)now_raw - (int32_t)last_raw;

    if (delta > AS5600_TICKS_PER_REV / 2) {
        delta -= AS5600_TICKS_PER_REV;
    } else if (delta < -(AS5600_TICKS_PER_REV / 2)) {
        delta += AS5600_TICKS_PER_REV;
    }

    return delta;
}

// ============================================================
// I2C MUX + AS5600
// ============================================================

static esp_err_t tca_select_channel(uint8_t channel)
{
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = (uint8_t)(1U << channel);
    return i2c_master_transmit(tca_handle, &data, 1, 100);
}

static esp_err_t as5600_read_raw(uint8_t channel, uint16_t *out_angle)
{
    if (out_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t ret = tca_select_channel(channel);
    if (ret == ESP_OK) {
        uint8_t reg = AS5600_REG_RAW;
        uint8_t buf[2] = {0};

        ret = i2c_master_transmit_receive(
            as5600_handle,
            &reg,
            1,
            buf,
            2,
            100
        );

        if (ret == ESP_OK) {
            // AS5600 RAW ANGLE is 12-bit across registers 0x0E and 0x0F.
            *out_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
        }
    }

    xSemaphoreGive(i2c_mutex);
    return ret;
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    i2c_device_config_t tca_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9548A_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        i2c_bus_handle,
        &tca_config,
        &tca_handle
    ));

    i2c_device_config_t as5600_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        i2c_bus_handle,
        &as5600_config,
        &as5600_handle
    ));

    i2c_device_config_t vl53_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        i2c_bus_handle,
        &vl53_config,
        &vl53_handle
    ));

    i2c_mutex = xSemaphoreCreateMutex();
    assert(i2c_mutex != NULL);

    ESP_LOGI(TAG, "I2C initialized: SDA=%d SCL=%d mux=0x%02X AS5600=0x%02X",
             I2C_SDA_PIN, I2C_SCL_PIN, TCA9548A_ADDR, AS5600_ADDR);
}

// ============================================================
// UART / RS485 TTL
// ============================================================

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT_NUM,
        UART_BUF_SIZE,
        0,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT_NUM,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "UART%d initialized: TX=%d RX=%d baud=%d",
             UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

// ============================================================
// VL53L0X TIME-OF-FLIGHT DRIVER
// ------------------------------------------------------------
// Four VL53L0X distance sensors, each on its own TCA9548A mux
// channel (they all share I2C address 0x29, so the mux is what
// distinguishes them). They run in continuous back-to-back
// ranging mode and are polled non-blocking. Range is reported
// in millimeters. Based on the canonical Pololu VL53L0X register
// sequence (single-shot ST API tuning settings).
// ============================================================

// VL53L0X registers (subset used here)
#define VL_SYSRANGE_START                 0x00
#define VL_SYSTEM_SEQUENCE_CONFIG         0x01
#define VL_SYSTEM_INTERRUPT_CONFIG_GPIO   0x0A
#define VL_SYSTEM_INTERRUPT_CLEAR         0x0B
#define VL_GPIO_HV_MUX_ACTIVE_HIGH        0x84
#define VL_RESULT_INTERRUPT_STATUS        0x13
#define VL_RESULT_RANGE_STATUS            0x14
#define VL_MSRC_CONFIG_CONTROL            0x60
#define VL_FINAL_RANGE_MIN_COUNT_RATE     0x44
#define VL_DYNAMIC_SPAD_REF_EN_START_OFF  0x4F
#define VL_DYNAMIC_SPAD_NUM_REQ_REF_SPAD  0x4E
#define VL_GLOBAL_CONFIG_REF_EN_START_SEL 0xB6
#define VL_GLOBAL_CONFIG_SPAD_ENABLES_0   0xB0
#define VL_VHV_CONFIG_PAD_SCL_SDA_EXTSUP  0x89
#define VL_IDENTIFICATION_MODEL_ID        0xC0

// Out-of-range / no-target readings come back near this value (mm).
#define VL_RANGE_OOR_MM                   8190

// Stored per mux channel: the device "stop variable" captured during init,
// needed to (re)start continuous ranging.
static uint8_t vl_stop_variable[8];

// Default tuning settings from the ST/Pololu reference init. {reg, value}.
static const uint8_t vl_tuning[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

// The vl_* helpers below assume the caller has already selected the correct
// mux channel AND holds i2c_mutex.

static esp_err_t vl_w8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(vl53_handle, buf, sizeof(buf), 100);
}

static esp_err_t vl_w16(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(vl53_handle, buf, sizeof(buf), 100);
}

static esp_err_t vl_r8(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(vl53_handle, &reg, 1, val, 1, 100);
}

static esp_err_t vl_r16(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(vl53_handle, &reg, 1, buf, 2, 100);
    if (ret == ESP_OK) {
        *val = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

static esp_err_t vl_read_multi(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(vl53_handle, &reg, 1, dst, n, 100);
}

static esp_err_t vl_write_multi(uint8_t reg, const uint8_t *src, size_t n)
{
    uint8_t buf[1 + 8];
    if (n > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    memcpy(&buf[1], src, n);
    return i2c_master_transmit(vl53_handle, buf, n + 1, 100);
}

static esp_err_t vl_get_spad_info(uint8_t *count, bool *is_aperture)
{
    uint8_t tmp = 0;

    vl_w8(0x80, 0x01);
    vl_w8(0xFF, 0x01);
    vl_w8(0x00, 0x00);
    vl_w8(0xFF, 0x06);
    vl_r8(0x83, &tmp);
    vl_w8(0x83, tmp | 0x04);
    vl_w8(0xFF, 0x07);
    vl_w8(0x81, 0x01);
    vl_w8(0x80, 0x01);
    vl_w8(0x94, 0x6b);
    vl_w8(0x83, 0x00);

    // Wait for the strobe (reg 0x83) to go non-zero.
    int64_t start = esp_timer_get_time();
    do {
        if (vl_r8(0x83, &tmp) != ESP_OK) {
            return ESP_FAIL;
        }
        if ((esp_timer_get_time() - start) > 100000) {  // 100 ms
            return ESP_ERR_TIMEOUT;
        }
    } while (tmp == 0x00);

    vl_w8(0x83, 0x01);
    vl_r8(0x92, &tmp);
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;

    vl_w8(0x81, 0x00);
    vl_w8(0xFF, 0x06);
    vl_r8(0x83, &tmp);
    vl_w8(0x83, tmp & ~0x04);
    vl_w8(0xFF, 0x01);
    vl_w8(0x00, 0x01);
    vl_w8(0xFF, 0x00);
    vl_w8(0x80, 0x00);

    return ESP_OK;
}

static esp_err_t vl_single_ref_cal(uint8_t vhv_init_byte)
{
    vl_w8(VL_SYSRANGE_START, 0x01 | vhv_init_byte);

    int64_t start = esp_timer_get_time();
    uint8_t status = 0;
    do {
        if (vl_r8(VL_RESULT_INTERRUPT_STATUS, &status) != ESP_OK) {
            return ESP_FAIL;
        }
        if ((esp_timer_get_time() - start) > 100000) {  // 100 ms
            return ESP_ERR_TIMEOUT;
        }
    } while ((status & 0x07) == 0);

    vl_w8(VL_SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl_w8(VL_SYSRANGE_START, 0x00);

    return ESP_OK;
}

static esp_err_t vl_init_locked(uint8_t channel)
{
    uint8_t tmp = 0;

    // Sanity: model ID should read 0xEE.
    if (vl_r8(VL_IDENTIFICATION_MODEL_ID, &tmp) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    if (tmp != 0xEE) {
        ESP_LOGW(TAG, "VL53L0X ch%d model ID 0x%02X (expected 0xEE)", channel, tmp);
    }

    // Use 2V8 mode for I/O voltage.
    vl_r8(VL_VHV_CONFIG_PAD_SCL_SDA_EXTSUP, &tmp);
    vl_w8(VL_VHV_CONFIG_PAD_SCL_SDA_EXTSUP, tmp | 0x01);

    // Set I2C standard mode and capture the stop variable.
    vl_w8(0x88, 0x00);
    vl_w8(0x80, 0x01);
    vl_w8(0xFF, 0x01);
    vl_w8(0x00, 0x00);
    vl_r8(0x91, &vl_stop_variable[channel]);
    vl_w8(0x00, 0x01);
    vl_w8(0xFF, 0x00);
    vl_w8(0x80, 0x00);

    // Disable MSRC and pre-range signal-rate-limit checks.
    vl_r8(VL_MSRC_CONFIG_CONTROL, &tmp);
    vl_w8(VL_MSRC_CONFIG_CONTROL, tmp | 0x12);

    // Final-range signal rate limit = 0.25 MCPS -> 0.25 * (1 << 7) = 32.
    vl_w16(VL_FINAL_RANGE_MIN_COUNT_RATE, 32);

    vl_w8(VL_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count = 0;
    bool spad_aperture = false;
    esp_err_t ret = vl_get_spad_info(&spad_count, &spad_aperture);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t spad_map[6] = {0};
    vl_read_multi(VL_GLOBAL_CONFIG_SPAD_ENABLES_0, spad_map, 6);

    vl_w8(0xFF, 0x01);
    vl_w8(VL_DYNAMIC_SPAD_REF_EN_START_OFF, 0x00);
    vl_w8(VL_DYNAMIC_SPAD_NUM_REQ_REF_SPAD, 0x2C);
    vl_w8(0xFF, 0x00);
    vl_w8(VL_GLOBAL_CONFIG_REF_EN_START_SEL, 0xB4);

    uint8_t first_spad = spad_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad || spads_enabled == spad_count) {
            spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }
    vl_write_multi(VL_GLOBAL_CONFIG_SPAD_ENABLES_0, spad_map, 6);

    // Load the default tuning settings.
    for (size_t i = 0; i < sizeof(vl_tuning) / sizeof(vl_tuning[0]); i++) {
        vl_w8(vl_tuning[i][0], vl_tuning[i][1]);
    }

    // Configure interrupt as "new sample ready", active low, then clear it.
    vl_w8(VL_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    vl_r8(VL_GPIO_HV_MUX_ACTIVE_HIGH, &tmp);
    vl_w8(VL_GPIO_HV_MUX_ACTIVE_HIGH, tmp & ~0x10);
    vl_w8(VL_SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Enabled sequence steps: DSS + pre-range + final-range.
    vl_w8(VL_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // Reference calibration (VHV then phase).
    vl_w8(VL_SYSTEM_SEQUENCE_CONFIG, 0x01);
    ret = vl_single_ref_cal(0x40);
    if (ret != ESP_OK) {
        return ret;
    }
    vl_w8(VL_SYSTEM_SEQUENCE_CONFIG, 0x02);
    ret = vl_single_ref_cal(0x00);
    if (ret != ESP_OK) {
        return ret;
    }
    vl_w8(VL_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return ESP_OK;
}

static esp_err_t vl_start_continuous_locked(uint8_t channel)
{
    vl_w8(0x80, 0x01);
    vl_w8(0xFF, 0x01);
    vl_w8(0x00, 0x00);
    vl_w8(0x91, vl_stop_variable[channel]);
    vl_w8(0x00, 0x01);
    vl_w8(0xFF, 0x00);
    vl_w8(0x80, 0x00);

    // 0x02 = continuous back-to-back ranging.
    return vl_w8(VL_SYSRANGE_START, 0x02);
}

// Initialize one TOF sensor and start it ranging. Takes the mutex and selects
// the mux channel internally.
static esp_err_t vl53l0x_init_channel(uint8_t channel)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t ret = tca_select_channel(channel);
    if (ret == ESP_OK) {
        ret = vl_init_locked(channel);
    }
    if (ret == ESP_OK) {
        ret = vl_start_continuous_locked(channel);
    }

    xSemaphoreGive(i2c_mutex);
    return ret;
}

// Non-blocking read of the latest continuous-mode range in mm. Returns
// ESP_ERR_TIMEOUT (without touching *mm) if no new sample is ready yet, so the
// caller can keep the previous value.
static esp_err_t vl53l0x_read_mm(uint8_t channel, uint16_t *mm)
{
    if (mm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t ret = tca_select_channel(channel);
    if (ret == ESP_OK) {
        uint8_t status = 0;
        ret = vl_r8(VL_RESULT_INTERRUPT_STATUS, &status);
        if (ret == ESP_OK) {
            if (status & 0x07) {
                ret = vl_r16(VL_RESULT_RANGE_STATUS + 10, mm);
                vl_w8(VL_SYSTEM_INTERRUPT_CLEAR, 0x01);
            } else {
                ret = ESP_ERR_TIMEOUT;  // no new sample yet
            }
        }
    }

    xSemaphoreGive(i2c_mutex);
    return ret;
}

// ============================================================
// MAIN
// ============================================================

void app_main(void)
{
    i2c_init();
    uart_init();

    // Initialize encoders so the first packet does not contain a fake jump.
    uint16_t enc1_last_raw = 0;
    uint16_t enc2_last_raw = 0;
    bool enc1_ready = false;
    bool enc2_ready = false;

    int init_attempts = 0;
    const int MAX_INIT_ATTEMPTS = 30;

    while ((!enc1_ready || !enc2_ready) && init_attempts < MAX_INIT_ATTEMPTS) {
        if (!enc1_ready) {
            if (as5600_read_raw(ENC1_CHANNEL, &enc1_last_raw) == ESP_OK) {
                enc1_ready = true;
                ESP_LOGI(TAG, "Vertical encoder (DENC1) initialized on mux channel %d raw=%u",
                         ENC1_CHANNEL, enc1_last_raw);
            } else {
                ESP_LOGD(TAG, "Waiting for vertical encoder on mux channel %d", ENC1_CHANNEL);
            }
        }

        if (!enc2_ready) {
            if (as5600_read_raw(ENC2_CHANNEL, &enc2_last_raw) == ESP_OK) {
                enc2_ready = true;
                ESP_LOGI(TAG, "Horizontal encoder (DENC2) initialized on mux channel %d raw=%u",
                         ENC2_CHANNEL, enc2_last_raw);
            } else {
                ESP_LOGD(TAG, "Waiting for horizontal encoder on mux channel %d", ENC2_CHANNEL);
            }
        }

        if (!enc1_ready || !enc2_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            init_attempts++;
        }
    }

    if (!enc1_ready) {
        ESP_LOGW(TAG, "Vertical encoder timeout, continuing without it");
    }
    if (!enc2_ready) {
        ESP_LOGW(TAG, "Horizontal encoder timeout, continuing without it");
    }

    // Initialize the four VL53L0X TOF sensors (one per mux channel) and start
    // continuous ranging. Sensors that fail init are skipped and report 0.
    // Order matches the packet field order: FRONT, RIGHT, LEFT, LBACK.
    const uint8_t tof_channels[4] = {
        TOF_FRONT_CH, TOF_RIGHT_CH, TOF_LEFT_CH, TOF_LBACK_CH
    };
    const char *tof_names[4] = { "FRONT", "RIGHT", "LEFT", "LBACK" };
    bool tof_ready[4] = { false, false, false, false };
    uint16_t tof_mm[4] = { 0, 0, 0, 0 };
    int tof_service_idx = 0;

    for (int i = 0; i < 4; i++) {
        esp_err_t ret = vl53l0x_init_channel(tof_channels[i]);
        if (ret == ESP_OK) {
            tof_ready[i] = true;
            ESP_LOGI(TAG, "TOF %s initialized on mux channel %d", tof_names[i], tof_channels[i]);
        } else {
            ESP_LOGW(TAG, "TOF %s init failed on mux channel %d: %s",
                     tof_names[i], tof_channels[i], esp_err_to_name(ret));
        }
    }

    int32_t enc1_delta_accum = 0;
    int32_t enc2_delta_accum = 0;

    int output_countdown = 0;

    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Streaming UART frames: DENC1=<ticks>,DENC2=<ticks>,H=<deg>,"
                  "TOF1..4=<in> (front,right,left,lback)");

    while (1) {
        // -------------------------
        // 1) Read encoders and accumulate deltas
        // -------------------------
        uint16_t enc1_raw = 0;
        uint16_t enc2_raw = 0;

        esp_err_t enc1_ret = as5600_read_raw(ENC1_CHANNEL, &enc1_raw);
        if (enc1_ret == ESP_OK) {
            int32_t d1 = wrap_encoder_delta(enc1_raw, enc1_last_raw);
            enc1_delta_accum += d1;
            enc1_last_raw = enc1_raw;
        } else {
            ESP_LOGD(TAG, "ENC1 read failed: %s", esp_err_to_name(enc1_ret));
        }

        esp_err_t enc2_ret = as5600_read_raw(ENC2_CHANNEL, &enc2_raw);
        if (enc2_ret == ESP_OK) {
            int32_t d2 = wrap_encoder_delta(enc2_raw, enc2_last_raw);
            enc2_delta_accum += d2;
            enc2_last_raw = enc2_raw;
        } else {
            ESP_LOGD(TAG, "ENC2 read failed: %s", esp_err_to_name(enc2_ret));
        }

        // -------------------------
        // 2) Service one TOF sensor per loop (round-robin, non-blocking).
        //    With 4 sensors at a 200 Hz loop, each updates at ~50 Hz, which
        //    comfortably keeps up with the VL53L0X ~30 Hz ranging rate.
        // -------------------------
        if (tof_ready[tof_service_idx]) {
            uint16_t mm = 0;
            if (vl53l0x_read_mm(tof_channels[tof_service_idx], &mm) == ESP_OK) {
                tof_mm[tof_service_idx] = mm;  // else: no new sample, keep last
            }
        }
        tof_service_idx = (tof_service_idx + 1) % 4;

        // -------------------------
        // 3) Output incremental packet over UART/RS485 at 50 Hz
        // -------------------------
        output_countdown++;
        if (output_countdown >= OUTPUT_DECIMATION) {
            output_countdown = 0;

            int32_t denc1_out = ENC1_SIGN * enc1_delta_accum;
            int32_t denc2_out = ENC2_SIGN * enc2_delta_accum;

            enc1_delta_accum = 0;
            enc2_delta_accum = 0;

            // Convert mm -> inches and apply the fixed offset (clamp at 0).
            float tof_in[4];
            for (int i = 0; i < 4; i++) {
                tof_in[i] = ((float)tof_mm[i] / MM_PER_INCH) - TOF_OFFSET_IN[i];
                if (tof_in[i] < 0.0f) {
                    tof_in[i] = 0.0f;
                }
            }

            char msg[160];
            int len = snprintf(
                msg,
                sizeof(msg),
                "DENC1=%ld,DENC2=%ld,H=%.4f,TOF1=%.2f,TOF2=%.2f,TOF3=%.2f,TOF4=%.2f\n",
                (long)denc1_out,
                (long)denc2_out,
                0.0f,        // H: constant 0 (no gyro on this board)
                tof_in[0],   // TOF1 = FRONT
                tof_in[1],   // TOF2 = RIGHT
                tof_in[2],   // TOF3 = LEFT
                tof_in[3]    // TOF4 = LBACK
            );

            if (len > 0) {
                uart_write_bytes(UART_PORT_NUM, msg, len);
            }

            ESP_LOGW(TAG, "DENC1=%ld DENC2=%ld H=0.0000 TOF1=%.2f TOF2=%.2f TOF3=%.2f TOF4=%.2f in",
                     (long)denc1_out, (long)denc2_out,
                     tof_in[0], tof_in[1], tof_in[2], tof_in[3]);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }
}
