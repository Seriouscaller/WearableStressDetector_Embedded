#include "board_config.h"
#include "types.h"
#include <stdio.h>

/* Device Configuration */
volatile bool is_sampling_active = true;

device_control_t device_config = {
    .show_telemetry = false,
    .show_logged_values = true,
    .show_battery_log = false,
    .show_gsr_debugging = false,
    .enable_ppg = true,
    .enable_gsr = true,
    .enable_imu = false,
    .enable_temp = false,
};
