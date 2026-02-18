#pragma once
#include <driver/i2c_master.h>

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z; 
} bmi160_data_t;

esp_err_t bmi160_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* dev_handle);
esp_err_t bmi160_read(i2c_master_dev_handle_t dev_handle, bmi160_data_t* bmi_data);
