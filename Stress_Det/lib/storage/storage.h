#pragma once

esp_err_t init_raw_data_ring_buffer(RingbufHandle_t *ring_buffer);
esp_err_t init_storage_transfer_learning(void);
void check_spiffs_status(const char *partition_name);