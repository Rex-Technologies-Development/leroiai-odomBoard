#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

// ============================================================
// ESP32-C6 ODOMETRY SENSOR BOARD
// ------------------------------------------------------------
// Inputs:
//   - TCA9548A 8-channel I2C mux
//   - AS5600 encoder #1 on mux channel 6
//   - AS5600 encoder #2 on mux channel 7
//   - ADXRS453Z single-axis gyro (Z/yaw) over SPI
//   - 4x VL53L0X time-of-flight distance sensors on mux channels 5,1,3,2
//
// Output over UART1 -> TTL side of RS485 module:
//   DENC1=<delta ticks>,DENC2=<delta ticks>,H=<heading deg>,
//   TOF1=<in>,TOF2=<in>,TOF3=<in>,TOF4=<in>
//
// Notes:
//   - DENC1/DENC2 are incremental deltas since the last UART packet,
//     not accumulated absolute encoder ticks.
//   - H is integrated gyro heading in degrees (Z-axis rotation only).
//     ADXRS453 is a single-axis rate gyro, integrated over time.
//   - TOF1..TOF4 are VL53L0X distances in inches, after a fixed offset
//     (see TOF_OFFSET_IN). Out-of-range / no-target reads come back near
//     321 in (~8190 mm).
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

// ADXRS453Z SPI
#define ADXRS_SPI_HOST      SPI2_HOST
#define ADXRS_SCK_PIN       GPIO_NUM_10
#define ADXRS_MOSI_PIN      GPIO_NUM_11
#define ADXRS_MISO_PIN      GPIO_NUM_5
#define ADXRS_CS_PIN        GPIO_NUM_18

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

// Your current mux wiring:
// Encoder 1: channel 6, physically closest to MCU.
// Encoder 2: channel 7.
#define ENC1_CHANNEL        6
#define ENC2_CHANNEL        7

// Per-encoder direction. Flip to -1 if a tracker's sign is backwards for
// the V5 brain's coordinate convention. Both encoders are flipped.
#define ENC1_SIGN           (-1)
#define ENC2_SIGN           (-1)

// Four VL53L0X TOF sensors -> packet fields TOF1..TOF4 in this order.
#define VL53L0X_CH_0        5
#define VL53L0X_CH_1        1
#define VL53L0X_CH_2        3
#define VL53L0X_CH_3        2

// TOF output is reported in inches. 1 inch = 25.4 mm.
#define MM_PER_INCH         25.4f

// Per-sensor fixed offset (inches) subtracted from each TOF reading to correct
// a constant optical-window setback. Right side (TOF1/TOF2) reads ~0.75" longer
// than the others, so it gets a larger offset.
//   TOF1/TOF2 (right): 15.75" -> 15.0" on top of the base 1.1"
//   TOF3/TOF4:         base ~1.1"
static const float TOF_OFFSET_IN[4] = { 1.85f, 1.85f, 1.1f, 1.1f };

// =====================
// UART SETTINGS
// =====================

#define UART_PORT_NUM       UART_NUM_1
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       2048

// =====================
// SPI / ADXRS453 SETTINGS
// =====================

// Start conservative. ADXRS453 supports a much higher max SPI clock,
// but 1 MHz is easier to debug on jumper wires.
#define ADXRS_SPI_CLOCK_HZ  1000000

// ADXRS453 rate register scale: 80 LSB per deg/s.
#define ADXRS_LSB_PER_DPS   80.0f

// Main sensor loop: 200 Hz. UART output: 50 Hz.
#define SENSOR_LOOP_HZ      200
#define SENSOR_LOOP_MS      (1000 / SENSOR_LOOP_HZ)
#define UART_OUTPUT_HZ      50
#define OUTPUT_DECIMATION   (SENSOR_LOOP_HZ / UART_OUTPUT_HZ)

// =====================
// HEADING TUNING
// =====================

// Change to +1.0f if your heading sign is backwards.
// With the old code, heading sign was -1, so I kept that convention.
#define GYRO_SIGN           1.0f

// Keep this small. ADXRS453 is already internally conditioned, so this
// is only to suppress tiny near-zero residuals after bias calibration.
#define GYRO_DEADBAND_DPS   0.02f

// Tune only if a known 360 deg physical turn does not report close to 360.
#define HEADING_SCALE       1.00f

// Startup bias calibration. Keep the robot still during this period.
// 1500 samples x 2 ms = ~3 s (under the 4 s power-up budget).
#define GYRO_BIAS_SAMPLES   1500
#define GYRO_BIAS_DELAY_MS  2

static const char *TAG = "ESP32C6_ODOM_RAW";

// =====================
// GLOBAL HANDLES
// =====================

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t tca_handle;
static i2c_master_dev_handle_t as5600_handle;
static i2c_master_dev_handle_t vl53_handle;
static SemaphoreHandle_t i2c_mutex;

static spi_device_handle_t adxrs_handle;

// =====================
// SMALL UTILS
// =====================

static float apply_deadband(float value, float deadband)
{
    if (value > -deadband && value < deadband) {
        return 0.0f;
    }
    return value;
}

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
// ADXRS453 SPI DRIVER
// ============================================================

static uint32_t adxrs453_add_odd_parity(uint32_t command)
{
    // Bit 0 is the command parity bit. It must make the whole 32-bit
    // command have odd parity.
    command &= ~1UL;

    uint8_t ones = 0;
    for (int bit = 31; bit >= 1; bit--) {
        ones += (uint8_t)((command >> bit) & 0x1);
    }

    if ((ones % 2) == 0) {
        command |= 1UL;
    }

    return command;
}

static esp_err_t adxrs453_transfer_u32(uint32_t tx_word, uint32_t *rx_word)
{
    uint8_t tx[4] = {
        (uint8_t)((tx_word >> 24) & 0xFF),
        (uint8_t)((tx_word >> 16) & 0xFF),
        (uint8_t)((tx_word >> 8) & 0xFF),
        (uint8_t)(tx_word & 0xFF),
    };

    uint8_t rx[4] = {0};

    spi_transaction_t trans = {
        .length = 32,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(adxrs_handle, &trans);
    if (ret != ESP_OK) {
        return ret;
    }

    if (rx_word != NULL) {
        *rx_word = ((uint32_t)rx[0] << 24) |
                   ((uint32_t)rx[1] << 16) |
                   ((uint32_t)rx[2] << 8)  |
                   ((uint32_t)rx[3]);
    }

    return ESP_OK;
}

static uint32_t adxrs453_make_read_command(uint8_t reg_addr)
{
    // Same command layout used by Analog Devices no-OS driver:
    // byte0 = READ bit | A8
    // byte1 = A7..A0 shifted left by 1
    uint8_t b0 = (uint8_t)(0x80 | (reg_addr >> 7));
    uint8_t b1 = (uint8_t)(reg_addr << 1);

    uint32_t cmd = ((uint32_t)b0 << 24) |
                   ((uint32_t)b1 << 16);

    return adxrs453_add_odd_parity(cmd);
}

static esp_err_t adxrs453_read_register16(uint8_t reg_addr, uint16_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t cmd = adxrs453_make_read_command(reg_addr);
    uint32_t resp = 0;

    // ADXRS453 is command/response pipelined. The response to this
    // read command comes back during the next SPI frame, so send twice.
    esp_err_t ret = adxrs453_transfer_u32(cmd, &resp);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = adxrs453_transfer_u32(cmd, &resp);
    if (ret != ESP_OK) {
        return ret;
    }

    // For a register read response, the 16-bit register value is positioned
    // like the Analog Devices no-OS driver extracts it:
    //   value = byte1<<11 | byte2<<3 | byte3>>5
    uint8_t b1 = (uint8_t)((resp >> 16) & 0xFF);
    uint8_t b2 = (uint8_t)((resp >> 8) & 0xFF);
    uint8_t b3 = (uint8_t)(resp & 0xFF);

    *value = ((uint16_t)b1 << 11) |
             ((uint16_t)b2 << 3)  |
             ((uint16_t)b3 >> 5);

    return ESP_OK;
}

static esp_err_t adxrs453_read_rate_dps(float *rate_dps)
{
    if (rate_dps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_u16 = 0;
    esp_err_t ret = adxrs453_read_register16(0x00, &raw_u16); // RATE1/RATE0
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_i16 = (int16_t)raw_u16;
    *rate_dps = ((float)raw_i16) / ADXRS_LSB_PER_DPS;

    return ESP_OK;
}

static void spi_init_adxrs453(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ADXRS_MOSI_PIN,
        .miso_io_num = ADXRS_MISO_PIN,
        .sclk_io_num = ADXRS_SCK_PIN,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(ADXRS_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ADXRS_SPI_CLOCK_HZ,
        .mode = 0,                 // ADXRS453: CPOL=0, CPHA=0
        .spics_io_num = ADXRS_CS_PIN,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(ADXRS_SPI_HOST, &dev_cfg, &adxrs_handle));

    // Datasheet startup recommendation: allow internal circuitry to initialize.
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ADXRS453 SPI initialized: SCK=%d MOSI=%d MISO=%d CS=%d clk=%d Hz",
             ADXRS_SCK_PIN, ADXRS_MOSI_PIN, ADXRS_MISO_PIN,
             ADXRS_CS_PIN, ADXRS_SPI_CLOCK_HZ);

    // Optional sanity check: PID register high byte is expected to begin with 0x52.
    uint16_t pid = 0;
    esp_err_t ret = adxrs453_read_register16(0x0C, &pid);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADXRS453 PID register raw=0x%04X", pid);
        if ((pid >> 8) != 0x52) {
            ESP_LOGW(TAG, "ADXRS453 PID high byte was not 0x52. Check SPI wiring/mode if rate is wrong.");
        }
    } else {
        ESP_LOGW(TAG, "Could not read ADXRS453 PID register: %s", esp_err_to_name(ret));
    }
}

static float calibrate_gyro_bias_dps(int samples, int delay_ms)
{
    ESP_LOGI(TAG, "Calibrating ADXRS453 gyro bias (%d samples). Keep board still.",
             samples);

    float sum = 0.0f;
    int good = 0;

    for (int i = 0; i < samples; i++) {
        float rate = 0.0f;
        esp_err_t ret = adxrs453_read_rate_dps(&rate);

        if (ret == ESP_OK) {
            sum += rate;
            good++;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    if (good == 0) {
        ESP_LOGW(TAG, "No valid gyro samples during bias calibration. Using 0 dps bias.");
        return 0.0f;
    }

    float bias = sum / (float)good;
    ESP_LOGI(TAG, "Gyro bias = %.6f deg/s from %d samples", bias, good);

    return bias;
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
// USB-C COMMAND INTERFACE (USB Serial/JTAG)
// ------------------------------------------------------------
// The native USB-C port is a secondary (output-only) console, so the
// USB Serial/JTAG peripheral is free for us to read commands from the
// Jetson. Single-character command:
//   'R' (or 'Z') -> reset the gyro heading to 0 (instant, heading only)
// ============================================================

static void usb_cmd_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t ret = usb_serial_jtag_driver_install(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "USB Serial/JTAG command input unavailable: %s",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "USB Serial/JTAG command input ready (R=reset gyro heading)");
    }
}

// Non-blocking. Scans any bytes the Jetson sent and returns the last
// recognized command character, or 0 if none. Tolerates line endings and
// whole words (e.g. "RESET\n" matches 'R', "ZERO\n" matches 'Z').
static char usb_cmd_poll(void)
{
    uint8_t buf[32];
    int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    char cmd = 0;

    for (int i = 0; i < n; i++) {
        char c = (char)buf[i];
        if (c == 'Z' || c == 'z' || c == 'R' || c == 'r') {
            cmd = c;  // last recognized command in this batch wins
        }
    }

    return cmd;
}

// ============================================================
// MAIN
// ============================================================

void app_main(void)
{
    i2c_init();
    uart_init();
    spi_init_adxrs453();
    usb_cmd_init();

    float gyro_bias_dps = calibrate_gyro_bias_dps(GYRO_BIAS_SAMPLES, GYRO_BIAS_DELAY_MS);

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
                ESP_LOGI(TAG, "Encoder 1 initialized on mux channel %d raw=%u",
                         ENC1_CHANNEL, enc1_last_raw);
            } else {
                ESP_LOGD(TAG, "Waiting for Encoder 1 on mux channel %d", ENC1_CHANNEL);
            }
        }

        if (!enc2_ready) {
            if (as5600_read_raw(ENC2_CHANNEL, &enc2_last_raw) == ESP_OK) {
                enc2_ready = true;
                ESP_LOGI(TAG, "Encoder 2 initialized on mux channel %d raw=%u",
                         ENC2_CHANNEL, enc2_last_raw);
            } else {
                ESP_LOGD(TAG, "Waiting for Encoder 2 on mux channel %d", ENC2_CHANNEL);
            }
        }

        if (!enc1_ready || !enc2_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            init_attempts++;
        }
    }

    if (!enc1_ready) {
        ESP_LOGW(TAG, "Encoder 1 timeout, continuing without it");
    }
    if (!enc2_ready) {
        ESP_LOGW(TAG, "Encoder 2 timeout, continuing without it");
    }

    // Initialize the four VL53L0X TOF sensors (one per mux channel) and start
    // continuous ranging. Sensors that fail init are skipped and report 0 mm.
    const uint8_t tof_channels[4] = {
        VL53L0X_CH_0, VL53L0X_CH_1, VL53L0X_CH_2, VL53L0X_CH_3
    };
    bool tof_ready[4] = { false, false, false, false };
    uint16_t tof_mm[4] = { 0, 0, 0, 0 };
    int tof_service_idx = 0;

    for (int i = 0; i < 4; i++) {
        esp_err_t ret = vl53l0x_init_channel(tof_channels[i]);
        if (ret == ESP_OK) {
            tof_ready[i] = true;
            ESP_LOGI(TAG, "TOF%d initialized on mux channel %d", i + 1, tof_channels[i]);
        } else {
            ESP_LOGW(TAG, "TOF%d init failed on mux channel %d: %s",
                     i + 1, tof_channels[i], esp_err_to_name(ret));
        }
    }

    float heading_deg = 0.0f;
    float last_rate_dps = 0.0f;
    bool have_last_rate = false;

    int32_t enc1_delta_accum = 0;
    int32_t enc2_delta_accum = 0;

    int output_countdown = 0;
    int64_t last_loop_time_us = esp_timer_get_time();

    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Streaming UART frames: DENC1=<ticks>,DENC2=<ticks>,H=<deg>,"
                  "TOF1..4=<in>");

    while (1) {
        // -------------------------
        // 0) Handle gyro-reset commands from the Jetson over USB-C
        // -------------------------
        char cmd = usb_cmd_poll();
        if (cmd == 'R' || cmd == 'r' || cmd == 'Z' || cmd == 'z') {
            heading_deg = 0.0f;
            have_last_rate = false;
            ESP_LOGW(TAG, "CMD %c: gyro heading reset to 0", cmd);
        }

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
        // 2) Read gyro rate and integrate heading
        // -------------------------
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_loop_time_us) / 1000000.0f;
        last_loop_time_us = now_us;

        // Avoid weird integration if the loop is paused by debugging/logging.
        if (dt < 0.0f || dt > 0.100f) {
            dt = (float)SENSOR_LOOP_MS / 1000.0f;
        }

        float rate_dps = 0.0f;
        esp_err_t gyro_ret = adxrs453_read_rate_dps(&rate_dps);

        if (gyro_ret == ESP_OK) {
            rate_dps = (rate_dps - gyro_bias_dps) * GYRO_SIGN * HEADING_SCALE;
            rate_dps = apply_deadband(rate_dps, GYRO_DEADBAND_DPS);

            if (!have_last_rate) {
                last_rate_dps = rate_dps;
                have_last_rate = true;
            }

            // Trapezoidal integration of angular rate.
            heading_deg += 0.5f * (last_rate_dps + rate_dps) * dt;
            last_rate_dps = rate_dps;
        } else {
            ESP_LOGW(TAG, "ADXRS453 rate read failed: %s", esp_err_to_name(gyro_ret));
        }

        // -------------------------
        // 3) Service one TOF sensor per loop (round-robin, non-blocking).
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
        // 4) Output incremental packet over UART/RS485 at 50 Hz
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
                heading_deg,
                tof_in[0],
                tof_in[1],
                tof_in[2],
                tof_in[3]
            );

            if (len > 0) {
                uart_write_bytes(UART_PORT_NUM, msg, len);
            }

            ESP_LOGW(TAG, "DENC1=%ld DENC2=%ld H=%.4f° TOF=[%.2f %.2f %.2f %.2f] in",
                     (long)denc1_out, (long)denc2_out, heading_deg,
                     tof_in[0], tof_in[1], tof_in[2], tof_in[3]);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }
}
