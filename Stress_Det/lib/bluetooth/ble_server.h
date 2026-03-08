#pragma once
#include <stdio.h>
#include "ble_server.h"
#include "services/gatt/ble_svc_gatt.h"
#include "gatt.h"

extern uint16_t sensor_chr_val_handle;

void init_ble_server(void);
int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg);
