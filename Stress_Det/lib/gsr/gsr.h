#pragma once
#include "driver/spi_master.h"
#include "esp_log.h"

esp_err_t gsr_sensor_init(spi_device_handle_t *gsr_handle);
esp_err_t gsr_sensor_read_raw(spi_device_handle_t handle, uint16_t *raw);
