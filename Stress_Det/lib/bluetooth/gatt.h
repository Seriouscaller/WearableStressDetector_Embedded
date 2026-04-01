#pragma once
#include "esp_log.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "types.h"
#include <stdio.h>

extern uint16_t ble_sensor_chr_a_val_handle;
extern uint16_t ble_sensor_chr_b_val_handle;
extern uint16_t ble_sensor_chr_c_val_handle;
