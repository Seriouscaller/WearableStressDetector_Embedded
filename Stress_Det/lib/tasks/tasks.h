#pragma once

void sensor_sampling_task(void *pvParameters);
void feature_extraction_task(void *pvParameters);
void logging_task(void *pvParameters);
void ble_update_task(void *pvParameters);
