#pragma once
#include "types.h"
#include <driver/i2c_master.h>

esp_err_t bmi160_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle);
esp_err_t bmi160_read(i2c_master_dev_handle_t dev_handle, bmi_data_t *bmi_data);
