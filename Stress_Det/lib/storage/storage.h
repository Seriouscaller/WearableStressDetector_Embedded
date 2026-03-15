#pragma once

void init_psram_buffer();
void sync_heartbeat_task(void *pvParameters);
void print_buffer_status_task(void *pvParameters);
void storage_task(void *pvParameters);