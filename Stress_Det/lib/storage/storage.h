#pragma once

esp_err_t init_psram_buffer(psram_ring_buffer_t *ring_buffer, uint32_t samples_count);
void sync_heartbeat_task(void *pvParameters);
void print_buffer_status_task(void *pvParameters);
void storage_task(void *pvParameters);