#include "board_config.h"
#include "types.h"
#include <stdio.h>

/* Device Configuration */
volatile bool is_sampling_active = true;

device_control_t device_config = {
    .show_telemetry = true,
    .show_logged_values = false,
    .show_battery_log = false,
    .show_gsr_debugging = false,
    .show_spiff_status = false,
    .enable_ppg = true,
    .enable_gsr = true,
    .enable_imu = true,
    .enable_temp = true,
};
