#pragma once
#include "driver/spi_master.h"
#include "esp_log.h"

#define GSR_R_FIXED 100000.0f;
#define GSR_V_REF   3.3f;

esp_err_t gsr_sensor_init(spi_device_handle_t *gsr_handle);
esp_err_t gsr_sensor_read_raw(spi_device_handle_t handle, uint16_t* raw);
