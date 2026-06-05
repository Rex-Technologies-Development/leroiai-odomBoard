// #include <stdio.h>
// #include <string.h>
// #include <stdint.h>
// #include <math.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"

// #include "driver/gpio.h"
// #include "driver/uart.h"
// #include "driver/i2c_master.h"

// #include "esp_log.h"
// #include "esp_err.h"
// #include "esp_timer.h"
// #include "esp_rom_sys.h"
// #include "esp_random.h"

// // =====================
// // PIN SETTINGS
// // =====================

// #define I2C_SDA_PIN GPIO_NUM_6
// #define I2C_SCL_PIN GPIO_NUM_7

// #define UART_TX_PIN GPIO_NUM_17
// #define UART_RX_PIN GPIO_NUM_16

// // =====================
// // DEVICE SETTINGS
// // =====================

// #define I2C_PORT_NUM        I2C_NUM_0
// #define I2C_FREQ_HZ         400000

// #define TCA9548A_ADDR       0x70
// #define TCA_CHANNEL         0

// #define WITMOTION_ADDR      0x50

// #define AS5600_ADDR         0x36
// #define AS5600_REG_RAW      0x0E
// #define AS5600_TICKS_PER_REV 4096

// #define ENC1_CHANNEL        7
// #define ENC2_CHANNEL        6

// // Same priority as app_main so FreeRTOS time-slices both fairly.
// // Higher priorities starve app_main because the encoder I2C calls block
// // for only microseconds at a time.
// #define ENCODER_TASK_PRIORITY 1
// #define ENCODER_TASK_STACK    4096

// // =====================
// // ODOMETRY
// // =====================
// // Frame: X right (+), Y forward (+), heading 0 = +Y, CW positive.
// // ENC2 = vertical tracker (forward+), ENC1 = horizontal tracker (right+).
// // Pick a unit (inches or meters) for WHEEL_DIAMETER_* and stay consistent;
// // pose X/Y will be in that same unit.

// #define WHEEL_DIAMETER_VERT     -2.0f   // vertical (forward) tracker wheel
// #define WHEEL_DIAMETER_HORIZ    -2.0f   // horizontal (right) tracker wheel

// // Signed offsets from the robot's rotation center to each tracking wheel.
// // VERT_OFFSET  = right-distance from center to the vertical tracker
// // HORIZ_OFFSET = forward-distance from center to the horizontal tracker
// // Set to 0 if the tracker passes through the rotation center.
// #define VERT_OFFSET             -5.125f
// #define HORIZ_OFFSET            -0.375f

// #define VERT_DIST_PER_TICK      ((float)M_PI * WHEEL_DIAMETER_VERT  / 4096.0f)
// #define HORIZ_DIST_PER_TICK     ((float)M_PI * WHEEL_DIAMETER_HORIZ / 4096.0f)

// #define ODOM_TASK_PERIOD_MS     10
// #define ODOM_TASK_PRIORITY      2
// #define ODOM_TASK_STACK         4096

// #define UART_PORT_NUM       UART_NUM_1
// #define UART_BAUD_RATE      115200

// // 50 Hz = 20 ms
// #define LOOP_PERIOD_US      20000

// // =====================
// // HEADING TUNING
// // =====================

// // Try 0.05 first. If it drifts while still, raise to 0.10 or 0.15.
// #define GYRO_DEADBAND       0.01f

// // If heading is consistently too small/big, tune this later.
// #define HEADING_SCALE       1.00f

// // Change this after testing:
// // 0 = use GX heading
// // 1 = use GY heading
// // 2 = use GZ heading
// #define HEADING_AXIS        2

// // Change to -1.0f if heading goes the wrong direction
// #define HEADING_SIGN        -1.0f

// static const char *TAG = "ODOMETRY_BOARD";

// static i2c_master_bus_handle_t i2c_bus_handle;
// static i2c_master_dev_handle_t tca_handle;
// static i2c_master_dev_handle_t wit_handle;
// static i2c_master_dev_handle_t as5600_handle;
// static SemaphoreHandle_t i2c_mutex;

// typedef struct {
//     uint16_t last_raw;
//     int32_t  ticks;
//     int8_t   direction;
//     bool     init_done;
// } encoder_state_t;

// static encoder_state_t enc1;
// static encoder_state_t enc2;
// static portMUX_TYPE enc_spinlock = portMUX_INITIALIZER_UNLOCKED;

// static float pose_x = 0.0f;     // world X, right positive
// static float pose_y = 0.0f;     // world Y, forward positive
// static float pose_theta = 0.0f; // radians, 0 = +Y, CW positive
// static portMUX_TYPE pose_spinlock = portMUX_INITIALIZER_UNLOCKED;

// // Heading in radians, written by app_main, read by odom_task.
// // Aligned 32-bit float reads/writes are atomic on this single-core MCU,
// // so a snapshot doesn't need a lock — volatile prevents register caching.
// static volatile float current_heading_rad = 0.0f;

// static float gx_bias = 0.0f;
// static float gy_bias = 0.0f;
// static float gz_bias = 0.0f;

// static float hx = 0.0f;
// static float hy = 0.0f;
// static float hz = 0.0f;

// static int64_t last_time_us = 0;

// static int16_t read_i16_le(uint8_t low, uint8_t high)
// {
//     return (int16_t)((high << 8) | low);
// }

// static esp_err_t tca_select_channel(uint8_t channel)
// {
//     if (channel > 7) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     uint8_t data = 1 << channel;
//     return i2c_master_transmit(tca_handle, &data, 1, 100);
// }

// static esp_err_t wit_read_registers(uint8_t start_reg, uint8_t *data, size_t len)
// {
//     xSemaphoreTake(i2c_mutex, portMAX_DELAY);

//     esp_err_t ret = tca_select_channel(TCA_CHANNEL);
//     if (ret == ESP_OK) {
//         ret = i2c_master_transmit_receive(
//             wit_handle,
//             &start_reg,
//             1,
//             data,
//             len,
//             100
//         );
//     }

//     xSemaphoreGive(i2c_mutex);
//     return ret;
// }

// static esp_err_t as5600_read_raw(uint8_t channel, uint16_t *out_angle)
// {
//     xSemaphoreTake(i2c_mutex, portMAX_DELAY);

//     esp_err_t ret = tca_select_channel(channel);
//     if (ret == ESP_OK) {
//         uint8_t reg = AS5600_REG_RAW;
//         uint8_t buf[2];
//         ret = i2c_master_transmit_receive(
//             as5600_handle, &reg, 1, buf, 2, 100
//         );
//         if (ret == ESP_OK) {
//             *out_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
//         }
//     }

//     xSemaphoreGive(i2c_mutex);
//     return ret;
// }

// static void encoder_update(encoder_state_t *enc, uint16_t raw)
// {
//     portENTER_CRITICAL(&enc_spinlock);
//     if (!enc->init_done) {
//         enc->last_raw = raw;
//         enc->init_done = true;
//         portEXIT_CRITICAL(&enc_spinlock);
//         return;
//     }

//     int32_t delta = (int32_t)raw - (int32_t)enc->last_raw;
//     if (delta > AS5600_TICKS_PER_REV / 2) {
//         delta -= AS5600_TICKS_PER_REV;
//     } else if (delta < -AS5600_TICKS_PER_REV / 2) {
//         delta += AS5600_TICKS_PER_REV;
//     }

//     enc->ticks += delta;
//     if (delta > 0)      enc->direction = 1;
//     else if (delta < 0) enc->direction = -1;
//     enc->last_raw = raw;
//     portEXIT_CRITICAL(&enc_spinlock);
// }

// static void encoder_snapshot(encoder_state_t *enc,
//                              int32_t *ticks, int8_t *direction, uint16_t *raw)
// {
//     portENTER_CRITICAL(&enc_spinlock);
//     *ticks = enc->ticks;
//     *direction = enc->direction;
//     *raw = enc->last_raw;
//     portEXIT_CRITICAL(&enc_spinlock);
// }

// static void encoder_task(void *arg)
// {
//     while (1) {
//         uint16_t raw;
//         if (as5600_read_raw(ENC1_CHANNEL, &raw) == ESP_OK) {
//             encoder_update(&enc1, raw);
//         }
//         if (as5600_read_raw(ENC2_CHANNEL, &raw) == ESP_OK) {
//             encoder_update(&enc2, raw);
//         }
//         // I2C waits yield naturally; this keeps the idle task fed so the
//         // task watchdog doesn't fire if a transaction completes very fast.
//         taskYIELD();
//     }
// }

// static void pose_snapshot(float *x, float *y, float *theta_rad)
// {
//     portENTER_CRITICAL(&pose_spinlock);
//     *x = pose_x;
//     *y = pose_y;
//     *theta_rad = pose_theta;
//     portEXIT_CRITICAL(&pose_spinlock);
// }

// static void odom_task(void *arg)
// {
//     // Wait briefly so app_main has populated current_heading_rad
//     // and the encoder task has filled in initial tick values.
//     vTaskDelay(pdMS_TO_TICKS(50));

//     int32_t  last_e1, last_e2;
//     int8_t   tmp_dir;
//     uint16_t tmp_raw;
//     encoder_snapshot(&enc1, &last_e1, &tmp_dir, &tmp_raw);
//     encoder_snapshot(&enc2, &last_e2, &tmp_dir, &tmp_raw);
//     float last_theta = current_heading_rad;

//     TickType_t last_wake = xTaskGetTickCount();
//     while (1) {
//         int32_t e1, e2;
//         encoder_snapshot(&enc1, &e1, &tmp_dir, &tmp_raw);
//         encoder_snapshot(&enc2, &e2, &tmp_dir, &tmp_raw);
//         float curr_theta = current_heading_rad;

//         int32_t d_e1 = e1 - last_e1;  // horizontal ticks (right positive)
//         int32_t d_e2 = e2 - last_e2;  // vertical ticks (forward positive)
//         float d_theta = curr_theta - last_theta;

//         float d_horiz = (float)d_e1 * HORIZ_DIST_PER_TICK; // body right
//         float d_vert  = (float)d_e2 * VERT_DIST_PER_TICK;  // body forward

//         float avg_theta = last_theta + d_theta * 0.5f;

//         float local_x, local_y;
//         if (fabsf(d_theta) < 1e-4f) {
//             local_x = d_horiz;
//             local_y = d_vert;
//         } else {
//             // Chord-of-arc correction with tracker offsets baked in.
//             // HORIZ_OFFSET is forward-distance to horizontal wheel: a wheel
//             // ahead of center moves rightward during CW rotation, so the
//             // measured d_horiz must have HORIZ_OFFSET*d_theta subtracted.
//             // VERT_OFFSET is right-distance to vertical wheel: a wheel right
//             // of center moves backward during CW rotation, so VERT_OFFSET*
//             // d_theta is added back.
//             float k = 2.0f * sinf(d_theta * 0.5f) / d_theta;
//             local_x = k * (d_horiz - HORIZ_OFFSET * d_theta);
//             local_y = k * (d_vert  + VERT_OFFSET  * d_theta);
//         }

//         // Body -> world: 0 rad faces +Y, theta increases CW.
//         float dx_world =  local_x * cosf(avg_theta) + local_y * sinf(avg_theta);
//         float dy_world = -local_x * sinf(avg_theta) + local_y * cosf(avg_theta);

//         portENTER_CRITICAL(&pose_spinlock);
//         pose_x += dx_world;
//         pose_y += dy_world;
//         pose_theta = curr_theta;
//         portEXIT_CRITICAL(&pose_spinlock);

//         last_e1 = e1;
//         last_e2 = e2;
//         last_theta = curr_theta;

//         vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ODOM_TASK_PERIOD_MS));
//     }
// }

// static void i2c_init(void)
// {
//     i2c_master_bus_config_t bus_config = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = I2C_PORT_NUM,
//         .scl_io_num = I2C_SCL_PIN,
//         .sda_io_num = I2C_SDA_PIN,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };

//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

//     i2c_device_config_t tca_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = TCA9548A_ADDR,
//         .scl_speed_hz = I2C_FREQ_HZ,
//     };

//     ESP_ERROR_CHECK(i2c_master_bus_add_device(
//         i2c_bus_handle,
//         &tca_config,
//         &tca_handle
//     ));

//     i2c_device_config_t wit_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = WITMOTION_ADDR,
//         .scl_speed_hz = I2C_FREQ_HZ,
//     };

//     ESP_ERROR_CHECK(i2c_master_bus_add_device(
//         i2c_bus_handle,
//         &wit_config,
//         &wit_handle
//     ));

//     i2c_device_config_t as5600_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = AS5600_ADDR,
//         .scl_speed_hz = I2C_FREQ_HZ,
//     };

//     ESP_ERROR_CHECK(i2c_master_bus_add_device(
//         i2c_bus_handle,
//         &as5600_config,
//         &as5600_handle
//     ));

//     i2c_mutex = xSemaphoreCreateMutex();
//     assert(i2c_mutex != NULL);

//     ESP_LOGI(TAG, "I2C initialized");
// }

// static void uart_init(void)
// {
//     uart_config_t uart_config = {
//         .baud_rate = UART_BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };

//     ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

//     ESP_ERROR_CHECK(uart_set_pin(
//         UART_PORT_NUM,
//         UART_TX_PIN,
//         UART_RX_PIN,
//         UART_PIN_NO_CHANGE,
//         UART_PIN_NO_CHANGE
//     ));

//     ESP_LOGI(TAG, "UART initialized");
// }

// static void calibrate_gyro_bias(void)
// {
//     printf("Calibrating gyro bias. Keep robot COMPLETELY still...\n");

//     const int samples = 300;
//     float gx_sum = 0.0f;
//     float gy_sum = 0.0f;
//     float gz_sum = 0.0f;
//     int good_samples = 0;

//     for (int i = 0; i < samples; i++) {
//         uint8_t data[24];

//         esp_err_t ret = wit_read_registers(0x34, data, sizeof(data));

//         if (ret == ESP_OK) {
//             int16_t gx_raw = read_i16_le(data[6], data[7]);
//             int16_t gy_raw = read_i16_le(data[8], data[9]);
//             int16_t gz_raw = read_i16_le(data[10], data[11]);

//             float gx = gx_raw / 32768.0f * 2000.0f;
//             float gy = gy_raw / 32768.0f * 2000.0f;
//             float gz = gz_raw / 32768.0f * 2000.0f;

//             gx_sum += gx;
//             gy_sum += gy;
//             gz_sum += gz;
//             good_samples++;
//         }

//         vTaskDelay(pdMS_TO_TICKS(5));
//     }

//     if (good_samples == 0) {
//         printf("WARNING: no good gyro samples. Bias set to 0.\n");
//         gx_bias = 0.0f;
//         gy_bias = 0.0f;
//         gz_bias = 0.0f;
//         return;
//     }

//     gx_bias = gx_sum / good_samples;
//     gy_bias = gy_sum / good_samples;
//     gz_bias = gz_sum / good_samples;

//     printf("Gyro bias from %d samples:\n", good_samples);
//     printf("GX bias = %.4f deg/s\n", gx_bias);
//     printf("GY bias = %.4f deg/s\n", gy_bias);
//     printf("GZ bias = %.4f deg/s\n", gz_bias);
//     printf("Heading axis = %d, sign = %.1f, scale = %.3f\n",
//            HEADING_AXIS, HEADING_SIGN, HEADING_SCALE);
// }

// static float apply_deadband(float value)
// {
//     if (value > -GYRO_DEADBAND && value < GYRO_DEADBAND) {
//         return 0.0f;
//     }

//     return value;
// }

// static void wait_until_next_loop(int64_t *next_loop_time_us)
// {
//     *next_loop_time_us += LOOP_PERIOD_US;

//     int64_t wait_us = *next_loop_time_us - esp_timer_get_time();

//     if (wait_us > 0) {
//         esp_rom_delay_us((uint32_t)wait_us);
//     } else {
//         *next_loop_time_us = esp_timer_get_time();
//     }
// }

// void app_main(void)
// {
//     i2c_init();
//     uart_init();

//     printf("ESP32-C6 odometry board started\n");

//     calibrate_gyro_bias();

//     hx = 0.0f;
//     hy = 0.0f;
//     hz = 0.0f;

//     xTaskCreate(encoder_task, "encoder_task", ENCODER_TASK_STACK,
//                 NULL, ENCODER_TASK_PRIORITY, NULL);

//     xTaskCreate(odom_task, "odom_task", ODOM_TASK_STACK,
//                 NULL, ODOM_TASK_PRIORITY, NULL);

//     last_time_us = esp_timer_get_time();
//     int64_t next_loop_time_us = esp_timer_get_time();

//     while (1) {
//         uint8_t data[24];

//         esp_err_t ret = wit_read_registers(0x34, data, sizeof(data));

//         if (ret != ESP_OK) {
//             char error_msg[96];

//             snprintf(error_msg, sizeof(error_msg),
//                      "ERR,WIT_READ_FAIL,%s\n",
//                      esp_err_to_name(ret));

//             uart_write_bytes(UART_PORT_NUM, error_msg, strlen(error_msg));
//             printf("%s", error_msg);

//             wait_until_next_loop(&next_loop_time_us);
//             continue;
//         }

//         int16_t ax_raw = read_i16_le(data[0], data[1]);
//         int16_t ay_raw = read_i16_le(data[2], data[3]);
//         int16_t az_raw = read_i16_le(data[4], data[5]);

//         int16_t gx_raw = read_i16_le(data[6], data[7]);
//         int16_t gy_raw = read_i16_le(data[8], data[9]);
//         int16_t gz_raw = read_i16_le(data[10], data[11]);

//         int16_t roll_raw  = read_i16_le(data[18], data[19]);
//         int16_t pitch_raw = read_i16_le(data[20], data[21]);
//         int16_t yaw_raw   = read_i16_le(data[22], data[23]);

//         float ax = ax_raw / 32768.0f * 16.0f;
//         float ay = ay_raw / 32768.0f * 16.0f;
//         float az = az_raw / 32768.0f * 16.0f;

//         float gx = gx_raw / 32768.0f * 2000.0f;
//         float gy = gy_raw / 32768.0f * 2000.0f;
//         float gz = gz_raw / 32768.0f * 2000.0f;

//         float roll  = roll_raw / 32768.0f * 180.0f;
//         float pitch = pitch_raw / 32768.0f * 180.0f;
//         float yaw   = yaw_raw / 32768.0f * 180.0f;

//         int64_t now_us = esp_timer_get_time();
//         float dt = (now_us - last_time_us) / 1000000.0f;
//         last_time_us = now_us;

//         float cgx = apply_deadband(gx - gx_bias);
//         float cgy = apply_deadband(gy - gy_bias);
//         float cgz = apply_deadband(gz - gz_bias);

//         hx += cgx * dt;
//         hy += cgy * dt;
//         hz += cgz * dt;

//         float selected_heading = 0.0f;

//         if (HEADING_AXIS == 0) {
//             selected_heading = hx;
//         } else if (HEADING_AXIS == 1) {
//             selected_heading = hy;
//         } else {
//             selected_heading = hz;
//         }

//         selected_heading = selected_heading * HEADING_SIGN * HEADING_SCALE;

//         // Publish heading in radians for the odometry task.
//         current_heading_rad = selected_heading * (float)(M_PI / 180.0);

//         int32_t  e1_ticks, e2_ticks;
//         int8_t   e1_dir, e2_dir;
//         uint16_t e1_raw, e2_raw;
//         encoder_snapshot(&enc1, &e1_ticks, &e1_dir, &e1_raw);
//         encoder_snapshot(&enc2, &e2_ticks, &e2_dir, &e2_raw);

//         float px, py, ptheta_rad;
//         pose_snapshot(&px, &py, &ptheta_rad);
//         float ptheta_deg = ptheta_rad * (float)(180.0 / M_PI);

//         char msg[384];

//         // snprintf(msg, sizeof(msg),
//         //          "IMU,H=%.2f,HX=%.2f,HY=%.2f,HZ=%.2f,AX=%.3f,AY=%.3f,AZ=%.3f,GX=%.2f,GY=%.2f,GZ=%.2f,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,ENC1=%ld,ENC2=%ld,X=%.3f,Y=%.3f,THETA=%.2f\n",
//         //          selected_heading,
//         //          hx,
//         //          hy,
//         //          hz,
//         //          ax,
//         //          ay,
//         //          az,
//         //          cgx,
//         //          cgy,
//         //          cgz,
//         //          roll,
//         //          pitch,
//         //          yaw,
//         //          (long)e1_ticks,
//         //          (long)e2_ticks,
//         //          px,
//         //          py,
//         //          ptheta_deg);

//         // snprintf(msg, sizeof(msg),
//         //          "X=%.3f,Y=%.3f,THETA=%.2f\n",
//         //          px,
//         //          py,
//         //          ptheta_deg);
//         //
//         // uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
//         // printf("%s", msg);

//         // Raw stream: encoder ticks + heading in degrees.
//         // The other device handles tick -> distance, heading -> radians,
//         // and runs the odometry math itself.
//         // snprintf(msg, sizeof(msg),
//         //          "ENC1=%ld,ENC2=%ld,H=%.4f\n",
//         //          (long)e1_ticks,
//         //          (long)e2_ticks,
//         //          selected_heading);
//         //
//         // uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
//         // printf("%s", msg);

//         // TEMP: random encoder values and random heading for receiver
//         // testing. Real ENC1/ENC2 still accumulate in the encoder task and
//         // the real heading is still in `selected_heading` — only the
//         // outgoing payload is faked. Revert by un-commenting the block
//         // above and commenting out this one (or restore main.c.bak).
//         int32_t rand_e1 = (int32_t)(esp_random() % 20001) - 10000;
//         int32_t rand_e2 = (int32_t)(esp_random() % 20001) - 10000;
//         float   rand_h  = ((float)esp_random() / (float)UINT32_MAX) * 720.0f - 360.0f;

//         snprintf(msg, sizeof(msg),
//                  "ENC1=%ld,ENC2=%ld,H=%.4f\n",
//                  (long)rand_e1,
//                  (long)rand_e2,
//                  rand_h);

//         uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
//         printf("%s", msg);

//         wait_until_next_loop(&next_loop_time_us);
//     }
// }

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// =====================
// PIN SETTINGS
// =====================

#define I2C_SDA_PIN GPIO_NUM_6
#define I2C_SCL_PIN GPIO_NUM_7

#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16

// =====================
// DEVICE SETTINGS
// =====================

#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_FREQ_HZ         400000

#define TCA9548A_ADDR       0x70
#define TCA_CHANNEL         0

#define WITMOTION_ADDR      0x50

#define AS5600_ADDR         0x36
#define AS5600_REG_RAW      0x0E
#define AS5600_TICKS_PER_REV 4096

#define ENC1_CHANNEL        3
#define ENC2_CHANNEL        2

// Same priority as app_main so FreeRTOS time-slices both fairly.
// Higher priorities starve app_main because the encoder I2C calls block
// for only microseconds at a time.
#define ENCODER_TASK_PRIORITY 1
#define ENCODER_TASK_STACK    4096

// =====================
// ODOMETRY
// =====================
// Frame: X right (+), Y forward (+), heading 0 = +Y, CW positive.
// ENC2 = vertical tracker (forward+), ENC1 = horizontal tracker (right+).
// Pick a unit (inches or meters) for WHEEL_DIAMETER_* and stay consistent;
// pose X/Y will be in that same unit.

#define WHEEL_DIAMETER_VERT     -2.0f   // vertical (forward) tracker wheel
#define WHEEL_DIAMETER_HORIZ    -2.0f   // horizontal (right) tracker wheel

// Signed offsets from the robot's rotation center to each tracking wheel.
// VERT_OFFSET  = right-distance from center to the vertical tracker
// HORIZ_OFFSET = forward-distance from center to the horizontal tracker
// Set to 0 if the tracker passes through the rotation center.
#define VERT_OFFSET             -5.125f
#define HORIZ_OFFSET            -0.375f

#define VERT_DIST_PER_TICK      ((float)M_PI * WHEEL_DIAMETER_VERT  / 4096.0f)
#define HORIZ_DIST_PER_TICK     ((float)M_PI * WHEEL_DIAMETER_HORIZ / 4096.0f)

#define ODOM_TASK_PERIOD_MS     10
#define ODOM_TASK_PRIORITY      2
#define ODOM_TASK_STACK         4096

#define UART_PORT_NUM       UART_NUM_1
#define UART_BAUD_RATE      115200

// 500 Hz IMU/integration loop (2 ms).
#define LOOP_PERIOD_US      2000

// Output every Nth IMU iteration. 10 -> UART at 50 Hz with the loop at 500 Hz.
#define OUTPUT_DECIMATION   10

// WitMotion digital low-pass filter setting written at init. Register 0x1F
// (BANDWIDTH). Values: 0=256Hz, 1=188Hz, 2=98Hz, 3=42Hz, 4=20Hz (default),
// 5=10Hz, 6=5Hz. We pick 0 (256Hz) so the LPF phase lag during fast yaw is
// negligible relative to a 500 Hz sample loop.
#define WIT_BANDWIDTH_REG       0x1F
#define WIT_BANDWIDTH_VALUE     0x0000

// Online gyro-bias refinement. When all three gyro axes (post-bias) stay
// below STILL_GYRO_THRESHOLD for STILL_REQUIRED_MS continuously, each axis'
// bias is nudged toward the current gyro reading with time-constant
// BIAS_FILTER_TAU_S. Bigger threshold = stricter stillness; bigger tau =
// slower bias adaptation.
#define STILL_GYRO_THRESHOLD    1.0f
#define STILL_REQUIRED_MS       500
#define BIAS_FILTER_TAU_S       1.0f
#define STILL_REQUIRED_SAMPLES  (STILL_REQUIRED_MS * 1000 / LOOP_PERIOD_US)

// =====================
// HEADING TUNING
// =====================

// Try 0.05 first. If it drifts while still, raise to 0.10 or 0.15.
#define GYRO_DEADBAND       0.01f

// If heading is consistently too small/big, tune this later.
#define HEADING_SCALE       1.00f

// Change this after testing:
// 0 = use GX heading
// 1 = use GY heading
// 2 = use GZ heading
#define HEADING_AXIS        2

// Change to -1.0f if heading goes the wrong direction
#define HEADING_SIGN        -1.0f

static const char *TAG = "ODOMETRY_BOARD";

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t tca_handle;
static i2c_master_dev_handle_t wit_handle;
static i2c_master_dev_handle_t as5600_handle;
static SemaphoreHandle_t i2c_mutex;

typedef struct {
    uint16_t last_raw;
    int32_t  ticks;
    int8_t   direction;
    bool     init_done;
} encoder_state_t;

static encoder_state_t enc1;
static encoder_state_t enc2;
static portMUX_TYPE enc_spinlock = portMUX_INITIALIZER_UNLOCKED;

static float pose_x = 0.0f;     // world X, right positive
static float pose_y = 0.0f;     // world Y, forward positive
static float pose_theta = 0.0f; // radians, 0 = +Y, CW positive
static portMUX_TYPE pose_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Heading in radians, written by app_main, read by odom_task.
// Aligned 32-bit float reads/writes are atomic on this single-core MCU,
// so a snapshot doesn't need a lock — volatile prevents register caching.
static volatile float current_heading_rad = 0.0f;

static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

static float hx = 0.0f;
static float hy = 0.0f;
static float hz = 0.0f;

static int64_t last_time_us = 0;

static int16_t read_i16_le(uint8_t low, uint8_t high)
{
    return (int16_t)((high << 8) | low);
}

static esp_err_t tca_select_channel(uint8_t channel)
{
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = 1 << channel;
    return i2c_master_transmit(tca_handle, &data, 1, 100);
}

static esp_err_t wit_read_registers(uint8_t start_reg, uint8_t *data, size_t len)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t ret = tca_select_channel(TCA_CHANNEL);
    if (ret == ESP_OK) {
        ret = i2c_master_transmit_receive(
            wit_handle,
            &start_reg,
            1,
            data,
            len,
            100
        );
    }

    xSemaphoreGive(i2c_mutex);
    return ret;
}

// WitMotion register write over I2C: <reg>, <low byte>, <high byte>.
// Little-endian 16-bit value.
static esp_err_t wit_write_register_16(uint8_t reg, uint16_t value)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t ret = tca_select_channel(TCA_CHANNEL);
    if (ret == ESP_OK) {
        uint8_t buf[3] = {
            reg,
            (uint8_t)(value & 0xFF),
            (uint8_t)((value >> 8) & 0xFF),
        };
        ret = i2c_master_transmit(wit_handle, buf, sizeof(buf), 100);
    }

    xSemaphoreGive(i2c_mutex);
    return ret;
}

static void wit_configure_lowpass(void)
{
    esp_err_t ret = wit_write_register_16(WIT_BANDWIDTH_REG, WIT_BANDWIDTH_VALUE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WitMotion BANDWIDTH write failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WitMotion gyro LPF set to max (256 Hz cutoff)");
    }
}

static esp_err_t as5600_read_raw(uint8_t channel, uint16_t *out_angle)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t ret = tca_select_channel(channel);
    if (ret == ESP_OK) {
        uint8_t reg = AS5600_REG_RAW;
        uint8_t buf[2];
        ret = i2c_master_transmit_receive(
            as5600_handle, &reg, 1, buf, 2, 100
        );
        if (ret == ESP_OK) {
            *out_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
        }
    }

    xSemaphoreGive(i2c_mutex);
    return ret;
}

static void encoder_update(encoder_state_t *enc, uint16_t raw)
{
    portENTER_CRITICAL(&enc_spinlock);
    if (!enc->init_done) {
        enc->last_raw = raw;
        enc->init_done = true;
        portEXIT_CRITICAL(&enc_spinlock);
        return;
    }

    int32_t delta = (int32_t)raw - (int32_t)enc->last_raw;
    if (delta > AS5600_TICKS_PER_REV / 2) {
        delta -= AS5600_TICKS_PER_REV;
    } else if (delta < -AS5600_TICKS_PER_REV / 2) {
        delta += AS5600_TICKS_PER_REV;
    }

    enc->ticks += delta;
    if (delta > 0)      enc->direction = 1;
    else if (delta < 0) enc->direction = -1;
    enc->last_raw = raw;
    portEXIT_CRITICAL(&enc_spinlock);
}

static void encoder_snapshot(encoder_state_t *enc,
                             int32_t *ticks, int8_t *direction, uint16_t *raw)
{
    portENTER_CRITICAL(&enc_spinlock);
    *ticks = enc->ticks;
    *direction = enc->direction;
    *raw = enc->last_raw;
    portEXIT_CRITICAL(&enc_spinlock);
}

static void encoder_task(void *arg)
{
    while (1) {
        uint16_t raw;
        if (as5600_read_raw(ENC1_CHANNEL, &raw) == ESP_OK) {
            encoder_update(&enc1, raw);
        }
        if (as5600_read_raw(ENC2_CHANNEL, &raw) == ESP_OK) {
            encoder_update(&enc2, raw);
        }
        // I2C waits yield naturally; this keeps the idle task fed so the
        // task watchdog doesn't fire if a transaction completes very fast.
        taskYIELD();
    }
}

static void pose_snapshot(float *x, float *y, float *theta_rad)
{
    portENTER_CRITICAL(&pose_spinlock);
    *x = pose_x;
    *y = pose_y;
    *theta_rad = pose_theta;
    portEXIT_CRITICAL(&pose_spinlock);
}

// static void odom_task(void *arg)
// {
//     // Wait briefly so app_main has populated current_heading_rad
//     // and the encoder task has filled in initial tick values.
//     vTaskDelay(pdMS_TO_TICKS(50));

//     int32_t  last_e1, last_e2;
//     int8_t   tmp_dir;
//     uint16_t tmp_raw;
//     encoder_snapshot(&enc1, &last_e1, &tmp_dir, &tmp_raw);
//     encoder_snapshot(&enc2, &last_e2, &tmp_dir, &tmp_raw);
//     float last_theta = current_heading_rad;

//     TickType_t last_wake = xTaskGetTickCount();
//     while (1) {
//         int32_t e1, e2;
//         encoder_snapshot(&enc1, &e1, &tmp_dir, &tmp_raw);
//         encoder_snapshot(&enc2, &e2, &tmp_dir, &tmp_raw);
//         float curr_theta = current_heading_rad;

//         int32_t d_e1 = e1 - last_e1;  // horizontal ticks (right positive)
//         int32_t d_e2 = e2 - last_e2;  // vertical ticks (forward positive)
//         float d_theta = curr_theta - last_theta;

//         float d_horiz = (float)d_e1 * HORIZ_DIST_PER_TICK; // body right
//         float d_vert  = (float)d_e2 * VERT_DIST_PER_TICK;  // body forward

//         float avg_theta = last_theta + d_theta * 0.5f;

//         float local_x, local_y;
//         if (fabsf(d_theta) < 1e-4f) {
//             local_x = d_horiz;
//             local_y = d_vert;
//         } else {
//             // Chord-of-arc correction with tracker offsets baked in.
//             // HORIZ_OFFSET is forward-distance to horizontal wheel: a wheel
//             // ahead of center moves rightward during CW rotation, so the
//             // measured d_horiz must have HORIZ_OFFSET*d_theta subtracted.
//             // VERT_OFFSET is right-distance to vertical wheel: a wheel right
//             // of center moves backward during CW rotation, so VERT_OFFSET*
//             // d_theta is added back.
//             float k = 2.0f * sinf(d_theta * 0.5f) / d_theta;
//             local_x = k * (d_horiz - HORIZ_OFFSET * d_theta);
//             local_y = k * (d_vert  + VERT_OFFSET  * d_theta);
//         }

//         // Body -> world: 0 rad faces +Y, theta increases CW.
//         float dx_world =  local_x * cosf(avg_theta) + local_y * sinf(avg_theta);
//         float dy_world = -local_x * sinf(avg_theta) + local_y * cosf(avg_theta);

//         portENTER_CRITICAL(&pose_spinlock);
//         pose_x += dx_world;
//         pose_y += dy_world;
//         pose_theta = curr_theta;
//         portEXIT_CRITICAL(&pose_spinlock);

//         last_e1 = e1;
//         last_e2 = e2;
//         last_theta = curr_theta;

//         vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ODOM_TASK_PERIOD_MS));
//     }
// }

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

    i2c_device_config_t wit_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = WITMOTION_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(
        i2c_bus_handle,
        &wit_config,
        &wit_handle
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

    ESP_LOGI(TAG, "I2C initialized");
}

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

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT_NUM,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "UART initialized");
}

static void calibrate_gyro_bias(void)
{
    printf("Calibrating gyro bias. Keep robot COMPLETELY still...\n");

    const int samples = 300;
    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;
    int good_samples = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t data[24];

        esp_err_t ret = wit_read_registers(0x34, data, sizeof(data));

        if (ret == ESP_OK) {
            int16_t gx_raw = read_i16_le(data[6], data[7]);
            int16_t gy_raw = read_i16_le(data[8], data[9]);
            int16_t gz_raw = read_i16_le(data[10], data[11]);

            float gx = gx_raw / 32768.0f * 2000.0f;
            float gy = gy_raw / 32768.0f * 2000.0f;
            float gz = gz_raw / 32768.0f * 2000.0f;

            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            good_samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (good_samples == 0) {
        printf("WARNING: no good gyro samples. Bias set to 0.\n");
        gx_bias = 0.0f;
        gy_bias = 0.0f;
        gz_bias = 0.0f;
        return;
    }

    gx_bias = gx_sum / good_samples;
    gy_bias = gy_sum / good_samples;
    gz_bias = gz_sum / good_samples;

    printf("Gyro bias from %d samples:\n", good_samples);
    printf("GX bias = %.4f deg/s\n", gx_bias);
    printf("GY bias = %.4f deg/s\n", gy_bias);
    printf("GZ bias = %.4f deg/s\n", gz_bias);
    printf("Heading axis = %d, sign = %.1f, scale = %.3f\n",
           HEADING_AXIS, HEADING_SIGN, HEADING_SCALE);
}

static float apply_deadband(float value)
{
    if (value > -GYRO_DEADBAND && value < GYRO_DEADBAND) {
        return 0.0f;
    }

    return value;
}

static void wait_until_next_loop(TickType_t *last_wake_time)
{
    vTaskDelayUntil(last_wake_time, pdMS_TO_TICKS(LOOP_PERIOD_US / 1000));
}

void app_main(void)
{
    i2c_init();
    uart_init();

    printf("ESP32-C6 odometry board started\n");

    // Widen the IMU's gyro LPF before calibration so the bias estimate is
    // measured under the same filter setting we'll use during operation.
    wit_configure_lowpass();
    // Give the IMU a moment to apply the new bandwidth before sampling.
    vTaskDelay(pdMS_TO_TICKS(50));

    calibrate_gyro_bias();

    hx = 0.0f;
    hy = 0.0f;
    hz = 0.0f;

    xTaskCreate(encoder_task, "encoder_task", ENCODER_TASK_STACK,
                NULL, ENCODER_TASK_PRIORITY, NULL);

    // xTaskCreate(odom_task, "odom_task", ODOM_TASK_STACK,
    //             NULL, ODOM_TASK_PRIORITY, NULL);

    last_time_us = esp_timer_get_time();
    TickType_t next_loop_time = xTaskGetTickCount();

    // Per-axis previous gyro reading for trapezoidal integration.
    static float last_cgx = 0.0f;
    static float last_cgy = 0.0f;
    static float last_cgz = 0.0f;
    // Consecutive samples observed stationary (bumped each iteration when
    // still, reset on motion; bias only adapts once it exceeds the
    // STILL_REQUIRED_SAMPLES threshold).
    static int still_count = 0;
    // Decimation counter for UART output.
    static int out_counter = 0;

    while (1) {
        uint8_t data[24];

        esp_err_t ret = wit_read_registers(0x34, data, sizeof(data));

        if (ret != ESP_OK) {
            char error_msg[96];

            snprintf(error_msg, sizeof(error_msg),
                     "ERR,WIT_READ_FAIL,%s\n",
                     esp_err_to_name(ret));

            uart_write_bytes(UART_PORT_NUM, error_msg, strlen(error_msg));
            printf("%s", error_msg);

            wait_until_next_loop(&next_loop_time);
            continue;
        }

        int16_t ax_raw = read_i16_le(data[0], data[1]);
        int16_t ay_raw = read_i16_le(data[2], data[3]);
        int16_t az_raw = read_i16_le(data[4], data[5]);

        int16_t gx_raw = read_i16_le(data[6], data[7]);
        int16_t gy_raw = read_i16_le(data[8], data[9]);
        int16_t gz_raw = read_i16_le(data[10], data[11]);

        int16_t roll_raw  = read_i16_le(data[18], data[19]);
        int16_t pitch_raw = read_i16_le(data[20], data[21]);
        int16_t yaw_raw   = read_i16_le(data[22], data[23]);

        float ax = ax_raw / 32768.0f * 16.0f;
        float ay = ay_raw / 32768.0f * 16.0f;
        float az = az_raw / 32768.0f * 16.0f;

        float gx = gx_raw / 32768.0f * 2000.0f;
        float gy = gy_raw / 32768.0f * 2000.0f;
        float gz = gz_raw / 32768.0f * 2000.0f;

        float roll  = roll_raw / 32768.0f * 180.0f;
        float pitch = pitch_raw / 32768.0f * 180.0f;
        float yaw   = yaw_raw / 32768.0f * 180.0f;

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;

        // Pre-deadband residual (post-bias) for stillness detection and
        // bias refinement. Honest noise check; we don't want the deadband
        // hiding small biases from the bias filter.
        float cgx_raw = gx - gx_bias;
        float cgy_raw = gy - gy_bias;
        float cgz_raw = gz - gz_bias;

        bool still = (fabsf(cgx_raw) < STILL_GYRO_THRESHOLD) &&
                     (fabsf(cgy_raw) < STILL_GYRO_THRESHOLD) &&
                     (fabsf(cgz_raw) < STILL_GYRO_THRESHOLD);

        if (still) {
            if (still_count < STILL_REQUIRED_SAMPLES) {
                still_count++;
            } else {
                // Stationary long enough: nudge bias toward the current
                // gyro reading. Gain = dt / tau gives a 1-tau exponential.
                float gain = dt / BIAS_FILTER_TAU_S;
                gx_bias += gain * cgx_raw;
                gy_bias += gain * cgy_raw;
                gz_bias += gain * cgz_raw;
            }
        } else {
            still_count = 0;
        }

        float cgx = apply_deadband(cgx_raw);
        float cgy = apply_deadband(cgy_raw);
        float cgz = apply_deadband(cgz_raw);

        // Trapezoidal integration:
        //   theta_new = theta_old + 0.5 * (omega_old + omega_new) * dt
        // Equivalent to the 2nd-order Taylor expansion using angular
        // acceleration alpha = (omega_new - omega_old)/dt.
        hx += 0.5f * (last_cgx + cgx) * dt;
        hy += 0.5f * (last_cgy + cgy) * dt;
        hz += 0.5f * (last_cgz + cgz) * dt;
        last_cgx = cgx;
        last_cgy = cgy;
        last_cgz = cgz;

        float selected_heading = 0.0f;

        if (HEADING_AXIS == 0) {
            selected_heading = hx;
        } else if (HEADING_AXIS == 1) {
            selected_heading = hy;
        } else {
            selected_heading = hz;
        }

        selected_heading = selected_heading * HEADING_SIGN * HEADING_SCALE;

        // Publish heading in radians for the odometry task.
        current_heading_rad = selected_heading * (float)(M_PI / 180.0);

        int32_t  e1_ticks, e2_ticks;
        int8_t   e1_dir, e2_dir;
        uint16_t e1_raw, e2_raw;
        encoder_snapshot(&enc1, &e1_ticks, &e1_dir, &e1_raw);
        encoder_snapshot(&enc2, &e2_ticks, &e2_dir, &e2_raw);

        float px, py, ptheta_rad;
        pose_snapshot(&px, &py, &ptheta_rad);
        float ptheta_deg = ptheta_rad * (float)(180.0 / M_PI);

        char msg[384];

        // snprintf(msg, sizeof(msg),
        //          "IMU,H=%.2f,HX=%.2f,HY=%.2f,HZ=%.2f,AX=%.3f,AY=%.3f,AZ=%.3f,GX=%.2f,GY=%.2f,GZ=%.2f,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,ENC1=%ld,ENC2=%ld,X=%.3f,Y=%.3f,THETA=%.2f\n",
        //          selected_heading,
        //          hx,
        //          hy,
        //          hz,
        //          ax,
        //          ay,
        //          az,
        //          cgx,
        //          cgy,
        //          cgz,
        //          roll,
        //          pitch,
        //          yaw,
        //          (long)e1_ticks,
        //          (long)e2_ticks,
        //          px,
        //          py,
        //          ptheta_deg);

        // snprintf(msg, sizeof(msg),
        //          "X=%.3f,Y=%.3f,THETA=%.2f\n",
        //          px,
        //          py,
        //          ptheta_deg);
        //
        // uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
        // printf("%s", msg);

        // Raw stream: encoder ticks + heading in degrees + timestamp (ms
        // since boot, for diagnosing pause/jitter — real pauses show as
        // gaps in T; pure monitor lag shows as bursts where T is regular).
        // Decimate so the 500 Hz integration loop only emits at 50 Hz.
        if (++out_counter >= OUTPUT_DECIMATION) {
            out_counter = 0;
            uint32_t ms_now = (uint32_t)(esp_timer_get_time() / 1000);
            snprintf(msg, sizeof(msg),
                     "T=%lu,ENC1=%ld,ENC2=%ld,H=%.4f\n",
                     (unsigned long)ms_now,
                     (long)e1_ticks,
                     (long)e2_ticks,
                     selected_heading);

            uart_write_bytes(UART_PORT_NUM, msg, strlen(msg));
            printf("%s", msg);
        }

        wait_until_next_loop(&next_loop_time);
    }
}