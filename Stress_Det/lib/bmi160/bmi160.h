#pragma once
#include <driver/i2c_master.h>
#include "types.h"

esp_err_t bmi160_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* dev_handle);
esp_err_t bmi160_read(i2c_master_dev_handle_t dev_handle, bmi160_data_t* bmi_data);
