#pragma once
#include "gatt.h"
#include <stdio.h>

int control_write_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg);
