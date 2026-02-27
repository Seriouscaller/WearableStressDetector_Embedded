#pragma once
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_random.h" // For generating random numbers
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

void ble_on_sync(void);
void ble_host_task(void *param);
void ble_update_sensor_data(uint16_t gsr, float temp, uint32_t ppg, int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x);
