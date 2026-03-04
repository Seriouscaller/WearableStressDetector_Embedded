#pragma once
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

extern const struct ble_gatt_svc_def gatt_svcs[];

extern uint16_t sensor_chr_val_handle;