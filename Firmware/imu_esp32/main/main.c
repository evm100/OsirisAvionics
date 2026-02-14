#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// --- Configuration ---
// Note: Check your ESP32-P4 board's datasheet for preferred I2C pins.
#define I2C_MASTER_SCL_IO           8     /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           7     /*!< GPIO number used for I2C master data */
#define I2C_MASTER_NUM              0     /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0     /*!< I2C master doesn't need buffer */

// --- LSM6DSOX Registers & Addresses ---
[span_7](start_span)#define LSM6DSOX_ADDR               0x6A  // Default I2C Address[span_7](end_span)
#define LSM6DSOX_WHO_AM_I           0x6C
#define LSM6DSOX_CTRL1_XL           0x10  // Accel Control
#define LSM6DSOX_CTRL2_G            0x11  // Gyro Control
#define LSM6DSOX_OUTX_L_G           0x22  // Gyro Output Start (Low byte X)
#define LSM6DSOX_OUTX_L_A           0x28  // Accel Output Start (Low byte X)

static const char *TAG = "LSM6DSOX";

/**
 * @brief Initialize the ESP32-P4 I2C Master Interface
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Write a byte to a sensor register
 */
static esp_err_t lsm6dsox_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSOX_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read multiple bytes starting from a register
 */
static esp_err_t lsm6dsox_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSOX_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSOX_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // 1. Check WHO_AM_I
    uint8_t who_am_i = 0;
    lsm6dsox_read_bytes(LSM6DSOX_WHO_AM_I, &who_am_i, 1);
    
    if (who_am_i != 0x6C) {
        ESP_LOGE(TAG, "LSM6DSOX not found! WHO_AM_I: 0x%02x (Expected 0x6C)", who_am_i);
        return;
    }
    ESP_LOGI(TAG, "LSM6DSOX found! WHO_AM_I: 0x%02x", who_am_i);

    // 2. Configure Sensor
    // CTRL1_XL (Accel): ODR=104Hz (0x40), Scale=2g (0x00) -> Total 0x40
    // CTRL2_G  (Gyro):  ODR=104Hz (0x40), Scale=250dps (0x00) -> Total 0x40
    [span_8](start_span)// See citation[span_8](end_span) for example data rates.
    lsm6dsox_write_byte(LSM6DSOX_CTRL1_XL, 0x40); 
    lsm6dsox_write_byte(LSM6DSOX_CTRL2_G, 0x40);

    ESP_LOGI(TAG, "Sensor configured for 104Hz, 2g, 250dps");

    while (1) {
        uint8_t raw_data[12];
        // Read 12 bytes: 6 for Gyro (starts at 0x22), 6 for Accel (starts at 0x28)
        // Note: In LSM6DSOX, Gyro registers (0x22-0x27) are typically before Accel (0x28-0x2D)
        
        // Read Gyro
        lsm6dsox_read_bytes(LSM6DSOX_OUTX_L_G, raw_data, 6);
        // Read Accel
        lsm6dsox_read_bytes(LSM6DSOX_OUTX_L_A, &raw_data[6], 6);

        int16_t gyro_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
        int16_t gyro_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
        int16_t gyro_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

        int16_t accel_x = (int16_t)((raw_data[7] << 8) | raw_data[6]);
        int16_t accel_y = (int16_t)((raw_data[9] << 8) | raw_data[8]);
        int16_t accel_z = (int16_t)((raw_data[11] << 8) | raw_data[10]);

        // Simple conversion for display (Approximation based on 2g/250dps settings)
        // 2g range sensitivity ~ 0.061 mg/LSB
        // 250dps range sensitivity ~ 8.75 mdps/LSB
        
        ESP_LOGI(TAG, "ACCEL [mg]: X=%5d Y=%5d Z=%5d | GYRO [mdps]: X=%5d Y=%5d Z=%5d", 
                 (int)(accel_x * 0.061), 
                 (int)(accel_y * 0.061), 
                 (int)(accel_z * 0.061),
                 (int)(gyro_x * 8.75), 
                 (int)(gyro_y * 8.75), 
                 (int)(gyro_z * 8.75));

        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay
    }
}
