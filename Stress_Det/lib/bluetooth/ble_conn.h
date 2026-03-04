#pragma once
#include <stdint.h>
#include "types.h"

void init_ble_conn(void);
void ble_conn_set_data(const app_sensor_data_t *data);