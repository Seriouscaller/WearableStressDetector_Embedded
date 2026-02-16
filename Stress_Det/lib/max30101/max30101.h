#pragma once
#include <driver/i2c_master.h>

typedef struct {
    uint32_t red_raw;
    uint32_t ir_raw;
    uint32_t green_raw; 
} max30101_data_t;

esp_err_t max30101_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* dev_handle);
esp_err_t max30101_read_fifo(i2c_master_dev_handle_t dev_handle, max30101_data_t* max_data);
