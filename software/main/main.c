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
// ESP32-C6 ODOMETRY SENSOR BOARD
// ------------------------------------------------------------
// Inputs:
//   - TCA9548A 8-channel I2C mux
//   - AS5600 encoder #1 on mux channel 4
//
// Output over UART1 -> TTL side of RS485 module:
//   DENC1=<delta ticks>
//
// Notes:
//   - DENC1 is the incremental delta since the last UART packet,
//     not the accumulated absolute encoder ticks.
// ============================================================

// =====================
// PIN SETTINGS
// =====================

// I2C mux bus
// NOTE: SDA/SCL temporarily SWAPPED as a wiring test (normally SDA=6, SCL=7).
#define I2C_SDA_PIN         GPIO_NUM_7
#define I2C_SCL_PIN         GPIO_NUM_6

// UART to TTL/RS485 module
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16

// =====================
// I2C DEVICE SETTINGS
// =====================

#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_FREQ_HZ         100000

#define TCA9548A_ADDR       0x70

#define AS5600_ADDR         0x36
#define AS5600_REG_RAW      0x0E
#define AS5600_TICKS_PER_REV 4096

// Mux wiring:
// Encoder 1: channel 4.
#define ENC1_CHANNEL        4

// Encoder direction. Flip to -1 if the tracker's sign is backwards for
// the V5 brain's coordinate convention.
#define ENC1_SIGN           (-1)

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

// One-shot diagnostic: separate the mux write from the AS5600 read so we can
// tell which device is NACKing, then sweep all 8 mux channels for an AS5600.
static void i2c_diag(void)
{
    // 0) Full 7-bit address scan directly on the bus (no mux selection).
    //    Tells us if ANYTHING is on the bus and at what address.
    ESP_LOGW(TAG, "DIAG: scanning I2C bus 0x08..0x77 ...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(i2c_bus_handle, addr, 50) == ESP_OK) {
            ESP_LOGW(TAG, "DIAG: *** device ACK at 0x%02X ***", addr);
            found++;
        }
    }
    ESP_LOGW(TAG, "DIAG: bus scan complete, %d device(s) responded", found);

    // 1) Probe the mux address directly on the bus.
    esp_err_t mux_probe = i2c_master_probe(i2c_bus_handle, TCA9548A_ADDR, 100);
    ESP_LOGW(TAG, "DIAG: mux probe 0x%02X = %s",
             TCA9548A_ADDR, esp_err_to_name(mux_probe));

    // 2) Split mux-select vs encoder-read on the configured channel.
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    uint8_t sel = (uint8_t)(1U << ENC1_CHANNEL);
    esp_err_t mux_ret = i2c_master_transmit(tca_handle, &sel, 1, 100);

    uint8_t reg = AS5600_REG_RAW;
    uint8_t buf[2] = {0};
    esp_err_t rd_ret = i2c_master_transmit_receive(
        as5600_handle, &reg, 1, buf, 2, 100);
    xSemaphoreGive(i2c_mutex);

    ESP_LOGW(TAG, "DIAG ch%d: mux write=%s | AS5600 read=%s buf=0x%02X%02X",
             ENC1_CHANNEL, esp_err_to_name(mux_ret),
             esp_err_to_name(rd_ret), buf[0], buf[1]);

    // 3) Sweep every channel looking for an AS5600 that answers.
    for (uint8_t ch = 0; ch < 8; ch++) {
        uint16_t raw = 0;
        esp_err_t ret = as5600_read_raw(ch, &raw);
        ESP_LOGW(TAG, "DIAG scan ch%d: %s raw=%u", ch, esp_err_to_name(ret), raw);
    }
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
// MAIN
// ============================================================

void app_main(void)
{
    i2c_init();
    uart_init();

    i2c_diag();

    // Initialize the encoder so the first packet does not contain a fake jump.
    uint16_t enc1_last_raw = 0;
    bool enc1_ready = false;

    int init_attempts = 0;
    const int MAX_INIT_ATTEMPTS = 30;

    while (!enc1_ready && init_attempts < MAX_INIT_ATTEMPTS) {
        if (as5600_read_raw(ENC1_CHANNEL, &enc1_last_raw) == ESP_OK) {
            enc1_ready = true;
            ESP_LOGI(TAG, "Encoder 1 initialized on mux channel %d raw=%u",
                     ENC1_CHANNEL, enc1_last_raw);
        } else {
            ESP_LOGD(TAG, "Waiting for Encoder 1 on mux channel %d", ENC1_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(100));
            init_attempts++;
        }
    }

    if (!enc1_ready) {
        ESP_LOGW(TAG, "Encoder 1 timeout, continuing without it");
    }

    int32_t enc1_delta_accum = 0;

    int output_countdown = 0;

    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Streaming UART frames: DENC1=<ticks>");

    while (1) {
        // -------------------------
        // 1) Read the encoder and accumulate deltas
        // -------------------------
        uint16_t enc1_raw = 0;

        esp_err_t enc1_ret = as5600_read_raw(ENC1_CHANNEL, &enc1_raw);
        if (enc1_ret == ESP_OK) {
            int32_t d1 = wrap_encoder_delta(enc1_raw, enc1_last_raw);
            enc1_delta_accum += d1;
            enc1_last_raw = enc1_raw;
        } else {
            ESP_LOGD(TAG, "ENC1 read failed: %s", esp_err_to_name(enc1_ret));
        }

        // DEBUG: raw AS5600 readout over I2C every output cycle.
        if (output_countdown == OUTPUT_DECIMATION - 1) {
            ESP_LOGW(TAG, "RAW ch%d ret=%s raw=%u (0x%03X) last=%u accum=%ld",
                     ENC1_CHANNEL, esp_err_to_name(enc1_ret),
                     enc1_raw, enc1_raw, enc1_last_raw, (long)enc1_delta_accum);
        }

        // -------------------------
        // 2) Output incremental packet over UART/RS485 at 50 Hz
        // -------------------------
        output_countdown++;
        if (output_countdown >= OUTPUT_DECIMATION) {
            output_countdown = 0;

            int32_t denc1_out = ENC1_SIGN * enc1_delta_accum;

            enc1_delta_accum = 0;

            char msg[64];
            int len = snprintf(
                msg,
                sizeof(msg),
                "DENC1=%ld\n",
                (long)denc1_out
            );

            if (len > 0) {
                uart_write_bytes(UART_PORT_NUM, msg, len);
            }

            ESP_LOGW(TAG, "DENC1=%ld", (long)denc1_out);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }
}
