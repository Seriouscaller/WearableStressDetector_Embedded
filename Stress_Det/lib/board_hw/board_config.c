#include "board_config.h"
#include "types.h"
#include <stdio.h>

/* Device Configuration */
bool show_telemetry = true;
bool show_logged_values = true;
bool show_gsr_debugging = false;
volatile bool is_sampling_active = true;

bool enable_ppg = true;
bool enable_gsr = true;
bool enable_imu = false;
bool enable_temp = false;