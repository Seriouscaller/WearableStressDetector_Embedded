#include "bmi160.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "gsr.h"
#include "i2c_common.h"
#include "max30101.h"
#include "spi_common.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "SENSOR_TASKS";
extern sensor_data_t ble_sensor_payload;
extern SemaphoreHandle_t sensor_data_mutex;
extern QueueHandle_t storage_queue;

// Sampling of IMU sensor every 10 ms (100 Hz).
// Adds read data to shared ble_sensor_payload struct
void imu_task(void *pvParameters)
{
    i2c_master_dev_handle_t bmi_handle = (i2c_master_dev_handle_t)pvParameters;
    bmi160_data_t imu_data = {0};

    while (1) {
        if (bmi160_read(bmi_handle, &imu_data) == ESP_OK) {
            // Update shared ble_sensor_payload with new IMU data. Mutex ensures that
            // only one task can access ble_sensor_payload at a time, preventing data corruption.
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.acc_x = imu_data.acc_x;
                ble_sensor_payload.acc_y = imu_data.acc_y;
                ble_sensor_payload.acc_z = imu_data.acc_z;
                ble_sensor_payload.gyr_x = imu_data.gyr_x;
                ble_sensor_payload.gyr_y = imu_data.gyr_y;
                ble_sensor_payload.gyr_z = imu_data.gyr_z;

                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Ax: %d Ay: %d Az: %d Gx: %d Gy: %d Gz: %d", imu_data.acc_x,
                         imu_data.acc_y, imu_data.acc_z, imu_data.gyr_x, imu_data.gyr_y,
                         imu_data.gyr_z);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read BMI160 sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLING_RATE_HZ));
    }
}

// Sampling of temperature sensor every 1000 ms (1 Hz).
// Adds read data to shared ble_sensor_payload struct
void temp_task(void *pvParameters)
{
    i2c_master_dev_handle_t tmp_handle = (i2c_master_dev_handle_t)pvParameters;
    float current_temp = 0.0f;

    while (1) {
        if (tmp117_read_temp(tmp_handle, &current_temp) == ESP_OK) {

            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.temp_raw = (uint16_t)(current_temp * 100);
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read temp: %.2f", current_temp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLING_RATE_HZ));
    }
}

// Sampling of PPG (blood) sensor every 20 ms (50 Hz).
// Adds read data to shared ble_sensor_payload struct
void ppg_task(void *pvParameters)
{
    i2c_master_dev_handle_t max_handle = (i2c_master_dev_handle_t)pvParameters;
    uint32_t current_ppg = 0;

    while (1) {
        if (max30101_read_fifo(max_handle, &current_ppg) == ESP_OK) {

            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.ppg_green = current_ppg;
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read PPG: %lu", current_ppg);
            }

        } else {
            ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(PPG_SAMPLING_RATE_HZ));
    }
}

// Sampling of GSR (sweat) sensor every 100 ms (10 Hz).
// Adds read data to shared ble_sensor_payload struct
void gsr_task(void *pvParameters)
{
    spi_device_handle_t gsr_handle = (spi_device_handle_t)pvParameters;
    uint16_t current_gsr = 0;

    while (1) {
        if (gsr_sensor_read_raw(gsr_handle, &current_gsr) == ESP_OK) {

            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.gsr = current_gsr;
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read GSR: %u", current_gsr);
            }

        } else {
            ESP_LOGW(TAG, "Failed to read GSR sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(GSR_SAMPLING_RATE_HZ));
    }
}