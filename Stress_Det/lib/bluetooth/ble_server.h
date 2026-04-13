#pragma once
#include "ble_server.h"
#include "gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include <stdio.h>

void init_ble_server(void);
int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg);
void ble_update_task(void *pvParameters);
int gatt_svr_dsc_access(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg);