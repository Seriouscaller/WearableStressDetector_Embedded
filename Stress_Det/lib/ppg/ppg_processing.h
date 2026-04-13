#pragma once
#include "ppg_hrv.h"
#include <stdio.h>

void ppg_processing_init(void);
float ppg_process_sample(uint32_t sample);
