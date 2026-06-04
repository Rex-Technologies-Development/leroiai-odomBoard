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
//   - ADXRS453Z gyro over SPI
//
// Output over UART1 -> TTL side of RS485 module:
//   DENC1=<delta ticks>,DENC2=<delta ticks>,H=<heading deg>
//
// Notes:
//   - DENC1/DENC2 are incremental deltas since the last UART packet,
//     not accumulated absolute encoder ticks.
//   - H is integrated gyro heading in degrees. ADXRS453 is a rate gyro,
//     so it does not provide absolute yaw by itself.
//   - VL53L0X sensors are not read in this version because the current
//     output requirement only uses encoder deltas + gyro heading.
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

// Your current mux wiring:
// Encoder 1: channel 6, physically closest to MCU.
// Encoder 2: channel 7.
#define ENC1_CHANNEL        6
#define ENC2_CHANNEL        7

// Present but unused for now.
#define VL53L0X_CH_0        5
#define VL53L0X_CH_1        1
#define VL53L0X_CH_2        3
#define VL53L0X_CH_3        2

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
#define GYRO_SIGN           -1.0f

// Keep this small. ADXRS453 is already internally conditioned, so this
// is only to suppress tiny near-zero residuals after bias calibration.
#define GYRO_DEADBAND_DPS   0.02f

// Tune only if a known 360 deg physical turn does not report close to 360.
#define HEADING_SCALE       1.00f

// Startup bias calibration. Keep the robot still during this period.
#define GYRO_BIAS_SAMPLES   400
#define GYRO_BIAS_DELAY_MS  5

static const char *TAG = "ESP32C6_ODOM_RAW";

// =====================
// GLOBAL HANDLES
// =====================

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t tca_handle;
static i2c_master_dev_handle_t as5600_handle;
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

static float calibrate_gyro_bias_dps(void)
{
    ESP_LOGI(TAG, "Calibrating ADXRS453 gyro bias. Keep robot completely still.");

    float sum = 0.0f;
    int good = 0;

    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
        float rate = 0.0f;
        esp_err_t ret = adxrs453_read_rate_dps(&rate);

        if (ret == ESP_OK) {
            sum += rate;
            good++;
        }

        vTaskDelay(pdMS_TO_TICKS(GYRO_BIAS_DELAY_MS));
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
// MAIN
// ============================================================

void app_main(void)
{
    i2c_init();
    uart_init();
    spi_init_adxrs453();

    float gyro_bias_dps = calibrate_gyro_bias_dps();

    // Initialize encoders so the first packet does not contain a fake jump.
    uint16_t enc1_last_raw = 0;
    uint16_t enc2_last_raw = 0;
    bool enc1_ready = false;
    bool enc2_ready = false;

    while (!enc1_ready || !enc2_ready) {
        if (!enc1_ready) {
            if (as5600_read_raw(ENC1_CHANNEL, &enc1_last_raw) == ESP_OK) {
                enc1_ready = true;
                ESP_LOGI(TAG, "Encoder 1 initialized on mux channel %d raw=%u",
                         ENC1_CHANNEL, enc1_last_raw);
            } else {
                ESP_LOGW(TAG, "Waiting for Encoder 1 on mux channel %d", ENC1_CHANNEL);
            }
        }

        if (!enc2_ready) {
            if (as5600_read_raw(ENC2_CHANNEL, &enc2_last_raw) == ESP_OK) {
                enc2_ready = true;
                ESP_LOGI(TAG, "Encoder 2 initialized on mux channel %d raw=%u",
                         ENC2_CHANNEL, enc2_last_raw);
            } else {
                ESP_LOGW(TAG, "Waiting for Encoder 2 on mux channel %d", ENC2_CHANNEL);
            }
        }

        if (!enc1_ready || !enc2_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
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

    ESP_LOGI(TAG, "Streaming UART frames: DENC1=<ticks>,DENC2=<ticks>,H=<deg>");

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
            ESP_LOGW(TAG, "ENC1 read failed: %s", esp_err_to_name(enc1_ret));
        }

        esp_err_t enc2_ret = as5600_read_raw(ENC2_CHANNEL, &enc2_raw);
        if (enc2_ret == ESP_OK) {
            int32_t d2 = wrap_encoder_delta(enc2_raw, enc2_last_raw);
            enc2_delta_accum += d2;
            enc2_last_raw = enc2_raw;
        } else {
            ESP_LOGW(TAG, "ENC2 read failed: %s", esp_err_to_name(enc2_ret));
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
        // 3) Output incremental packet over UART/RS485 at 50 Hz
        // -------------------------
        output_countdown++;
        if (output_countdown >= OUTPUT_DECIMATION) {
            output_countdown = 0;

            int32_t denc1_out = enc1_delta_accum;
            int32_t denc2_out = enc2_delta_accum;

            enc1_delta_accum = 0;
            enc2_delta_accum = 0;

            char msg[96];
            int len = snprintf(
                msg,
                sizeof(msg),
                "DENC1=%ld,DENC2=%ld,H=%.4f\n",
                (long)denc1_out,
                (long)denc2_out,
                heading_deg
            );

            if (len > 0) {
                uart_write_bytes(UART_PORT_NUM, msg, len);
            }

            ESP_LOGI(TAG, "Heading: %.4f°", heading_deg);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }
}
