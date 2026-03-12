// BMI160 IMU Acceleration & Gyroscope

#include "bmi160.h"
#include "i2c_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board_config.h"

static const char *TAG = "BMI160";

#define BMI160_ADDR           0x69
#define BMI160_CHIP_ID        0xD1
#define BMI160_REG_CHIP_ID    0x00
#define BMI160_REG_ERR        0x02
#define BMI160_REG_DATA       0x0C
#define BMI160_REG_STATUS     0x1B
#define BMI160_REG_ACC_CONF   0x40
#define BMI160_REG_ACC_RANGE  0x41
#define BMI160_REG_GYR_CONF   0x42
#define BMI160_REG_GYR_RANGE  0x43
#define BMI160_REG_CMD        0x7E

#define BMI160_CMD_SOFT_RESET 0xB6
#define BMI160_CMD_ACC_NORMAL 0x11
#define BMI160_CMD_GYR_NORMAL 0x15
#define BMI160_ACC_RANGE_2G   0x03
#define BMI160_GYR_RANGE_2000 0x00
#define BMI160_ACC_CONF_100HZ 0x28
#define BMI160_GYR_CONF_100HZ 0x28

// Adds device to i2c.
// Configures device.
esp_err_t bmi160_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* dev_handle){
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI160_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Adding device to i2c-bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    if(ret != ESP_OK ){
        ESP_LOGE(TAG, "Failed to add BMI160 to i2c");
        return ret;
    }

    // Check that sensor is the expected type/ID
    uint8_t chip_id_reg = BMI160_REG_CHIP_ID;
    uint8_t chip_id = 0;
    ret = i2c_master_transmit_receive(*dev_handle, &chip_id_reg, 1, &chip_id, 1, -1);
    if(ret != ESP_OK || chip_id != BMI160_CHIP_ID){
        ESP_LOGE(TAG, "Incorrect chip-id found: 0x%02X. Should be 0x%02X", chip_id, BMI160_CHIP_ID);
        return ESP_FAIL;
    }

    // Resets sensor
    ret = write_reg(*dev_handle, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
    if(ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set accel to Normal Mode
    ret = write_reg(*dev_handle, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    if(ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set gyro to Normal Mode
    ret = write_reg(*dev_handle, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
    if(ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Accelerometer range +-2g
    write_reg(*dev_handle, BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_2G);
    // Gyroscope range +- 2000 deg/s
    write_reg(*dev_handle, BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_2000);

    // Configure Accel: 100Hz, Normal power, 4 samples avg
    write_reg(*dev_handle, BMI160_REG_ACC_CONF, BMI160_ACC_CONF_100HZ);
    // Configure Gyro: 100Hz, Normal power
    write_reg(*dev_handle, BMI160_REG_GYR_CONF, BMI160_GYR_CONF_100HZ);

    ESP_LOGI(TAG, "Initialized successfully");
    return ESP_OK;
}

// Reads 12B total of IMU data from sensor. Converts from little-endian
// to big endian for correct data representation.
esp_err_t bmi160_read(i2c_master_dev_handle_t dev_handle, bmi160_data_t* data){
    uint8_t reg = BMI160_REG_DATA;
    uint8_t buffer[12]; 

    // Receives 6B Acceleration & 6B Gyroscope-data from sensor. Stored in buffer.
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 12, -1);

    if(ret == ESP_OK) {
        
        // Little endian storage converted to big endian for correct repr.
        data->gyr_x = (int16_t)((buffer[1] << 8)  | buffer[0]);
        data->gyr_y = (int16_t)((buffer[3] << 8)  | buffer[2]);
        data->gyr_z = (int16_t)((buffer[5] << 8)  | buffer[4]);

        data->acc_x = (int16_t)((buffer[7] << 8)  | buffer[6]);
        data->acc_y = (int16_t)((buffer[9] << 8)  | buffer[8]);
        data->acc_z = (int16_t)((buffer[11] << 8) | buffer[10]);
    }
    return ret;
}