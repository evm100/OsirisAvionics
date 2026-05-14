#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "BOOT_DEBUG";

// PINS
#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000

#define LSM6DSOX_ADDR               0x6A
#define BMP388_ADDR                 0x77

// Registers
#define LSM_REG_WHO_AM_I            0x0F
#define LSM_REG_CTRL1_XL            0x10
#define LSM_REG_CTRL2_G             0x11
#define LSM_REG_CTRL3_C             0x12
#define LSM_REG_STATUS              0x1E
#define LSM_REG_OUT_DATA            0x22

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

// Write a byte
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

// Read bytes
void read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void app_main(void) {
    i2c_master_init();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "--- REBOOTING SENSORS ---");

    // 1. SOFT RESET LSM6DSOX
    // Write 0x01 to CTRL3_C (SW_RESET)
    ESP_LOGW(TAG, "Resetting LSM6DSOX...");
    write_reg(LSM6DSOX_ADDR, LSM_REG_CTRL3_C, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reboot

    // 2. CONFIGURE LSM (Enable BDU + AutoIncrement)
    // CTRL3_C: BDU=1 (Block Data Update), IF_INC=1 (Auto Increment) -> 0x44
    write_reg(LSM6DSOX_ADDR, LSM_REG_CTRL3_C, 0x44); 
    
    // 3. ENABLE ACCEL & GYRO
    // CTRL1_XL: High Performance, 104Hz, 4g -> 0x4A (Wait... lets try 0x60 for 416Hz)
    write_reg(LSM6DSOX_ADDR, LSM_REG_CTRL1_XL, 0x60); 
    // CTRL2_G: High Performance, 104Hz, 2000dps -> 0x60
    write_reg(LSM6DSOX_ADDR, LSM_REG_CTRL2_G, 0x60);

    // 4. VERIFY CONFIGURATION (Did the write work?)
    uint8_t ctrl1=0, ctrl2=0;
    read_regs(LSM6DSOX_ADDR, LSM_REG_CTRL1_XL, &ctrl1, 1);
    read_regs(LSM6DSOX_ADDR, LSM_REG_CTRL2_G, &ctrl2, 1);
    
    if (ctrl1 == 0x60 && ctrl2 == 0x60) {
        ESP_LOGI(TAG, "OK LSM Config Verified: Active!");
    } else {
        ESP_LOGE(TAG, "X LSM Config FAILED! Read: 0x%02X 0x%02X (Expected 0x60 0x60)", ctrl1, ctrl2);
    }

    // 5. BMP Setup (Just keeping what worked)
    write_reg(BMP388_ADDR, 0x1B, 0x33);
    write_reg(BMP388_ADDR, 0x1D, 0x03);

    ESP_LOGI(TAG, "--- STARTING STREAM ---");
    while(1) {
        // Check Status
        uint8_t status = 0;
        read_regs(LSM6DSOX_ADDR, LSM_REG_STATUS, &status, 1);
        
        // Read Data
        uint8_t imu[12] = {0};
        read_regs(LSM6DSOX_ADDR, LSM_REG_OUT_DATA, imu, 12);

        // Print
        printf("STATUS: %02X | GyrX: %02X%02X | AccX: %02X%02X\n", 
               status, imu[1], imu[0], imu[7], imu[6]);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
