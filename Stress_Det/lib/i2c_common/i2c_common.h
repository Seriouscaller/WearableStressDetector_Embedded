#pragma once
#include "driver/i2c_master.h"
#include <stdint.h>

esp_err_t init_i2c(i2c_master_bus_handle_t *bus_handle);
esp_err_t write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data);
esp_err_t read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data);