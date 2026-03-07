/*
 * ESP32-P4 LSM6DSOX IMU Reader
 *
 * Reads accelerometer and gyroscope data from the LSM6DSOX over I2C
 * using the current ESP-IDF v5.x i2c_master API.
 *
 * Hardware: SDA=GPIO7, SCL=GPIO8
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "LSM6DSOX";

// --- I2C Configuration ---
#define I2C_MASTER_SDA_IO       7
#define I2C_MASTER_SCL_IO       8
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   1000

// --- LSM6DSOX Registers ---
#define LSM6DSOX_ADDR           0x6A
#define LSM6DSOX_WHO_AM_I_REG   0x0F
#define LSM6DSOX_WHO_AM_I_VAL   0x6C
#define LSM6DSOX_CTRL1_XL       0x10    // Accelerometer control
#define LSM6DSOX_CTRL2_G        0x11    // Gyroscope control
#define LSM6DSOX_CTRL3_C        0x12    // Control register 3 (SW_RESET, BDU)
#define LSM6DSOX_CTRL9_XL       0x18    // Control register 9 (I3C disable)
#define LSM6DSOX_OUTX_L_G       0x22    // Gyro output start
#define LSM6DSOX_OUTX_L_A       0x28    // Accel output start

// --- Sensitivity / Scaling ---
// Accel +/-4g: 0.122 mg/LSB -> m/s^2
#define ACCEL_SCALE             (0.122f * 9.80665f / 1000.0f)
// Gyro 250 dps: 8.75 mdps/LSB -> dps
#define GYRO_SCALE              (8.75f / 1000.0f)

// --- I2C Helper Functions ---

static esp_err_t lsm6dsox_register_read(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t lsm6dsox_register_write_byte(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}

// --- Sensor Initialization ---

static esp_err_t lsm6dsox_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t dev)
{
    esp_err_t ret;

    // Probe the device on the bus
    ret = i2c_master_probe(bus, LSM6DSOX_ADDR, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LSM6DSOX not found on bus (probe failed)");
        return ret;
    }

    // WHO_AM_I check
    uint8_t who_am_i = 0;
    ret = lsm6dsox_register_read(dev, LSM6DSOX_WHO_AM_I_REG, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    if (who_am_i != LSM6DSOX_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", who_am_i, LSM6DSOX_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (OK)", who_am_i);

    // Software reset (CTRL3_C bit 0)
    ret = lsm6dsox_register_write_byte(dev, LSM6DSOX_CTRL3_C, 0x01);
    if (ret != ESP_OK) return ret;

    // Poll until reset completes (bit 0 clears)
    uint8_t ctrl3 = 0x01;
    for (int i = 0; i < 100 && (ctrl3 & 0x01); i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        lsm6dsox_register_read(dev, LSM6DSOX_CTRL3_C, &ctrl3, 1);
    }
    if (ctrl3 & 0x01) {
        ESP_LOGE(TAG, "Software reset timed out");
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Software reset complete");

    // Enable Block Data Update (CTRL3_C bit 6)
    ret = lsm6dsox_register_read(dev, LSM6DSOX_CTRL3_C, &ctrl3, 1);
    if (ret != ESP_OK) return ret;
    ret = lsm6dsox_register_write_byte(dev, LSM6DSOX_CTRL3_C, ctrl3 | 0x40);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "Block Data Update enabled");

    // Disable I3C interface (CTRL9_XL bit 0)
    uint8_t ctrl9 = 0;
    ret = lsm6dsox_register_read(dev, LSM6DSOX_CTRL9_XL, &ctrl9, 1);
    if (ret != ESP_OK) return ret;
    ret = lsm6dsox_register_write_byte(dev, LSM6DSOX_CTRL9_XL, ctrl9 | 0x01);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "I3C interface disabled");

    // Configure accelerometer: ODR=104Hz, FS=+/-4g
    // CTRL1_XL: [7:4]=0100 (104Hz), [3:2]=10 (4g) => 0x48
    ret = lsm6dsox_register_write_byte(dev, LSM6DSOX_CTRL1_XL, 0x48);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(200));

    // Configure gyroscope: ODR=104Hz, FS=250dps
    // CTRL2_G: [7:4]=0100 (104Hz), [3:2]=00 (250dps) => 0x40
    ret = lsm6dsox_register_write_byte(dev, LSM6DSOX_CTRL2_G, 0x40);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Configured: Accel 104Hz/4g, Gyro 104Hz/250dps");
    return ESP_OK;
}

// --- Main Application ---

void app_main(void)
{
    // Initialize I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // Add LSM6DSOX as I2C device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DSOX_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    ESP_LOGI(TAG, "I2C master initialized (SDA=GPIO%d, SCL=GPIO%d, %dkHz)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ / 1000);

    // Initialize sensor
    esp_err_t ret = lsm6dsox_init(bus_handle, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed (0x%x)", ret);
        return;
    }

    // Data reading loop
    int error_count = 0;

    while (1) {
        uint8_t gyro_raw[6], accel_raw[6];

        // Read gyroscope (6 bytes from 0x22)
        ret = lsm6dsox_register_read(dev_handle, LSM6DSOX_OUTX_L_G, gyro_raw, 6);
        if (ret != ESP_OK) {
            error_count++;
            ESP_LOGW(TAG, "Gyro read failed (%d consecutive)", error_count);
            if (error_count >= 10) {
                ESP_LOGE(TAG, "Too many errors, resetting bus");
                i2c_master_bus_reset(bus_handle);
                error_count = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Read accelerometer (6 bytes from 0x28)
        ret = lsm6dsox_register_read(dev_handle, LSM6DSOX_OUTX_L_A, accel_raw, 6);
        if (ret != ESP_OK) {
            error_count++;
            ESP_LOGW(TAG, "Accel read failed (%d consecutive)", error_count);
            if (error_count >= 10) {
                ESP_LOGE(TAG, "Too many errors, resetting bus");
                i2c_master_bus_reset(bus_handle);
                error_count = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        error_count = 0;

        // Parse raw int16 little-endian values
        int16_t gx = (int16_t)((gyro_raw[1] << 8) | gyro_raw[0]);
        int16_t gy = (int16_t)((gyro_raw[3] << 8) | gyro_raw[2]);
        int16_t gz = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]);

        int16_t ax = (int16_t)((accel_raw[1] << 8) | accel_raw[0]);
        int16_t ay = (int16_t)((accel_raw[3] << 8) | accel_raw[2]);
        int16_t az = (int16_t)((accel_raw[5] << 8) | accel_raw[4]);

        // Convert to physical units
        float accel_x = ax * ACCEL_SCALE;
        float accel_y = ay * ACCEL_SCALE;
        float accel_z = az * ACCEL_SCALE;

        float gyro_x = gx * GYRO_SCALE;
        float gyro_y = gy * GYRO_SCALE;
        float gyro_z = gz * GYRO_SCALE;

        ESP_LOGI(TAG, "Accel [m/s2]: X=%+7.3f Y=%+7.3f Z=%+7.3f | Gyro [dps]: X=%+8.3f Y=%+8.3f Z=%+8.3f",
                 accel_x, accel_y, accel_z,
                 gyro_x, gyro_y, gyro_z);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
