#pragma once
#include <stdio.h>
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "types.h"

extern const ble_uuid128_t sensor_svc_uuid;
extern const ble_uuid128_t sensor_chr_uuid;

extern const struct ble_gatt_svc_def gatt_svcs[];