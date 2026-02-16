#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

void init_i2c(i2c_master_bus_handle_t* bus_handle);