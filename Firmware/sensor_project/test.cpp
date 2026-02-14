#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "RAW_DEBUG";

// ==========================================================
// PIN CONFIGURATION
// ==========================================================
#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000

// We know these are correct now:
#define LSM6DSOX_ADDR               0x6A 
#define BMP388_ADDR                 0x77 

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Helper to write a register
void write_reg(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Helper to read registers
void read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void app_main(void) {
    i2c_master_init();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "--- INITIALIZING SENSORS ---");

    // 1. WAKE UP LSM6DSOX
    // CTRL1_XL (0x10) = 0x60 (High Performance, 416Hz)
    write_reg(LSM6DSOX_ADDR, 0x10, 0x60); 
    // CTRL2_G (0x11) = 0x60 (High Performance, 416Hz)
    write_reg(LSM6DSOX_ADDR, 0x11, 0x60); 
    
    // 2. WAKE UP BMP388
    // PWR_CTRL (0x1B) = 0x33 (Press ON, Temp ON, Normal Mode)
    write_reg(BMP388_ADDR, 0x1B, 0x33);
    // ODR (0x1D) = 0x03 (50Hz)
    write_reg(BMP388_ADDR, 0x1D, 0x03);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 3. DEBUG BMP CALIBRATION
    // If this prints all 00s, your sensor is not reading correctly.
    uint8_t cal[10]; 
    read_regs(BMP388_ADDR, 0x31, cal, 10);
    ESP_LOGW(TAG, "BMP388 Calib Sample: %02X %02X %02X %02X (Should NOT be 00 00...)", cal[0], cal[1], cal[2], cal[3]);

    ESP_LOGI(TAG, "--- STARTING RAW DUMP ---");
    
    while(1) {
        // A. Check LSM6DSOX Status
        uint8_t lsm_status = 0;
        read_regs(LSM6DSOX_ADDR, 0x1E, &lsm_status, 1);
        
        // B. Read LSM6DSOX Data (12 bytes)
        uint8_t lsm_data[12];
        read_regs(LSM6DSOX_ADDR, 0x22, lsm_data, 12);

        // C. Read BMP388 Data (6 bytes)
        uint8_t bmp_data[6];
        read_regs(BMP388_ADDR, 0x04, bmp_data, 6);

        // PRINT RAW HEX
        // Status Bit 0 = Accel New Data, Bit 1 = Gyro New Data
        printf("STATUS: %02X | IMU: %02X %02X %02X | BMP: %02X %02X %02X\n", 
               lsm_status, 
               lsm_data[0], lsm_data[1], lsm_data[2], // Just showing first 3 bytes of IMU
               bmp_data[2], bmp_data[1], bmp_data[0]  // Pressure bytes
        );

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
