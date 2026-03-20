#pragma once
#include "types.h"

esp_err_t get_ppg_snapshot(psram_ppg_ring_buffer_t *ppg_buffer, uint32_t *dest_array);
void ppg_processing_task(void *pvParameters);
esp_err_t init_ppg_psram_buffer(psram_ppg_ring_buffer_t *ppg_buffer, uint32_t samples_count);
