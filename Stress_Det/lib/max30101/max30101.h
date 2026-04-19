#pragma once
#include "types.h"
#include <driver/i2c_master.h>

esp_err_t max30101_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle);
esp_err_t max30101_read_fifo(i2c_master_dev_handle_t dev_handle, uint32_t *max_data);
esp_err_t max30101_get_fifo_count(i2c_master_dev_handle_t dev_handle, uint8_t *count);