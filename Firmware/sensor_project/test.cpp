#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// ==========================================================
// PIN CONFIGURATION (DOUBLE CHECK THESE!)
// ==========================================================
#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000

// ADDRs (Updated based on your successful scan)
#define LSM6DSOX_ADDR               0x6A // or 0x6B
#define BMP388_ADDR                 0x76 // or 0x77

static const char *TAG = "DEBUG";

// Helper: Read Register
esp_err_t read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Helper: Write Register
esp_err_t write_reg(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

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

void app_main(void) {
    i2c_master_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for power up
    
    ESP_LOGW(TAG, "--- STARTING SENSOR DEBUG ---");

    // 1. INIT LSM6DSOX
    uint8_t lsm_id = 0;
    read_regs(LSM6DSOX_ADDR, 0x0F, &lsm_id, 1);
    ESP_LOGI(TAG, "LSM6DSOX ID: 0x%02X (Expect 0x6C)", lsm_id);

    // Reset LSM (Optional, but good practice)
    write_reg(LSM6DSOX_ADDR, 0x12, 0x01); // CTRL3_C -> SW_RESET
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Enable Accelerometer (52Hz, 4g)
    write_reg(LSM6DSOX_ADDR, 0x10, 0x38); 
    // Enable Gyro (52Hz, 2000dps)
    write_reg(LSM6DSOX_ADDR, 0x11, 0x3C);
    
    // 2. INIT BMP388
    uint8_t bmp_id = 0;
    read_regs(BMP388_ADDR, 0x00, &bmp_id, 1);
    ESP_LOGI(TAG, "BMP388 ID: 0x%02X (Expect 0x50)", bmp_id);
    
    // BMP388 Power ON Sequence
    write_reg(BMP388_ADDR, 0x1B, 0x33); // PWR_CTRL: Press ON, Temp ON, Mode Normal
    write_reg(BMP388_ADDR, 0x1D, 0x02); // ODR: 50Hz

    // 3. READ CALIBRATION (CRITICAL CHECK)
    uint8_t cal[21];
    esp_err_t ret = read_regs(BMP388_ADDR, 0x31, cal, 21);
    if (ret == ESP_OK) {
        printf("BMP Calibration Dump: ");
        for(int i=0; i<5; i++) printf("%02X ", cal[i]);
        printf("...\n");
        if (cal[0] == 0 && cal[1] == 0) {
            ESP_LOGE(TAG, "CRITICAL: Calibration data is ALL ZEROS. T=0.00 bug will occur.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read BMP Calibration!");
    }

    // MAIN LOOP: READ RAW HEX ONLY
    while (1) {
        uint8_t status_lsm = 0;
        read_regs(LSM6DSOX_ADDR, 0x1E, &status_lsm, 1); // STATUS_REG

        uint8_t raw_lsm[12];
        read_regs(LSM6DSOX_ADDR, 0x22, raw_lsm, 12); // Read Gyro(6) + Accel(6)

        uint8_t raw_bmp[6];
        read_regs(BMP388_ADDR, 0x04, raw_bmp, 6); // Pressure(3) + Temp(3)

        // PRINT RAW DATA
        // If these are all 00, sensor is sleeping. If they don't change, sensor is frozen.
        printf("LSM Status: 0x%02X | Gyr: %02X%02X %02X%02X %02X%02X | Acc: %02X%02X %02X%02X %02X%02X || BMP: %02X %02X %02X\n", 
               status_lsm,
               raw_lsm[1], raw_lsm[0], raw_lsm[3], raw_lsm[2], raw_lsm[5], raw_lsm[4], // Gyro X Y Z
               raw_lsm[7], raw_lsm[6], raw_lsm[9], raw_lsm[8], raw_lsm[11], raw_lsm[10], // Accel X Y Z
               raw_bmp[2], raw_bmp[1], raw_bmp[0] // Pressure MSB->LSB
               );
        
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
