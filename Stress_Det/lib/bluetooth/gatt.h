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

extern const ble_uuid128_t sensor_svc_uuid;
extern const ble_uuid128_t sensor_chr_uuid;

extern const struct ble_gatt_svc_def gatt_svcs[];