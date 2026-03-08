#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "driver/i2c_master.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_server.h"
#include "types.h"
#include "i2c_common.h"
#include "spi_common.h"
#include "max30101.h"
#include "tmp117.h"
#include "gsr.h"
#include "bmi160.h"

uint16_t conn_handle;
uint16_t sensor_chr_val_handle;
SemaphoreHandle_t sensor_data_mutex;
static const char *TAG = "MAIN";
sensor_data_t ble_sensor_payload = { .company_id = 0x02E5 }; // Example ID

i2c_master_dev_handle_t add_tmp117_i2c(i2c_master_bus_handle_t bus_handle){
    i2c_master_dev_handle_t tmp_handle;
    ESP_ERROR_CHECK(tmp117_init(bus_handle, &tmp_handle));
    return tmp_handle;
}

i2c_master_dev_handle_t add_max30101_i2c(i2c_master_bus_handle_t bus_handle){
    i2c_master_dev_handle_t max_handle;
    ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));
    return max_handle;
}

i2c_master_dev_handle_t add_bmi160_i2c(i2c_master_bus_handle_t bus_handle){
    i2c_master_dev_handle_t bmi_handle;
    ESP_ERROR_CHECK(bmi160_init(bus_handle, &bmi_handle));
    return bmi_handle;
}

spi_device_handle_t add_gsr_spi() {
    spi_device_handle_t gsr_handle;
    ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));
    return gsr_handle;
}

void display_raw_ppg(i2c_master_dev_handle_t max_handle, uint32_t* ppg_green){
    if (max30101_read_fifo(max_handle, ppg_green) == ESP_OK) {
        printf(">PPG_Green_Raw:%lu\n", *ppg_green);
    } else {
        ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
    }
}

void display_temperature(i2c_master_dev_handle_t tmp_handle, float* temperature){
    esp_err_t ret = tmp117_read_temp(tmp_handle, temperature);
    if(ret == ESP_OK){
        printf(">Temperature:%.2f\n", *temperature);
    } else {
        ESP_LOGW(TAG, "Failed to read TMP117 sensor.");
    }
}

void display_raw_imu(i2c_master_dev_handle_t imu_handle, bmi160_data_t* data){
    
    esp_err_t ret = bmi160_read(imu_handle, data);
    if(ret == ESP_OK){
        printf(">Acc x:%d\n", data->acc_x);
        printf(">Acc y:%d\n", data->acc_y);
        printf(">Acc z:%d\n", data->acc_z);

        printf(">Gyr x:%d\n", data->gyr_x);
        printf(">Gyr y:%d\n", data->gyr_y);
        printf(">Gyr z:%d\n", data->gyr_z);
    } else {
        ESP_LOGW(TAG, "Failed to read BMI160 sensor.");
    }
}

void display_raw_gsr(spi_device_handle_t gsr_handle, uint16_t* raw_gsr){

    if (gsr_sensor_read_raw(gsr_handle, raw_gsr) == ESP_OK) {
        printf(">Raw GSR:%u\n", *raw_gsr);
    } else {
        ESP_LOGW(TAG, "Failed to read GSR sensor.");
    }
}

void imu_task(void *pvParameters){
    i2c_master_dev_handle_t bmi_handle = (i2c_master_dev_handle_t)pvParameters;
    bmi160_data_t imu_data = {0};

    while (1) {
        if(bmi160_read(bmi_handle, &imu_data) == ESP_OK){
            
            if(xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
                ble_sensor_payload.acc_x = imu_data.acc_x;
                ble_sensor_payload.acc_y = imu_data.acc_y;
                ble_sensor_payload.acc_z = imu_data.acc_z;
                ble_sensor_payload.gyr_x = imu_data.gyr_x;
                ble_sensor_payload.gyr_y = imu_data.gyr_y;
                ble_sensor_payload.gyr_z = imu_data.gyr_z;

                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Ax: %d Ay: %d Az: %d Gx: %d Gy: %d Gz: %d", 
                    imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                    imu_data.gyr_x, imu_data.gyr_y, imu_data.gyr_z);
            }
        }else {
            ESP_LOGW(TAG, "Failed to read BMI160 sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void temp_task(void *pvParameters){
    i2c_master_dev_handle_t tmp_handle = (i2c_master_dev_handle_t)pvParameters;
    float current_temp = 0.0f;

    while (1) {
        if(tmp117_read_temp(tmp_handle, &current_temp) == ESP_OK){
            
            if(xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
                ble_sensor_payload.temp_raw = (uint16_t)(current_temp * 100);
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read temp: %.2f", current_temp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ppg_task(void *pvParameters){
    i2c_master_dev_handle_t max_handle = (i2c_master_dev_handle_t)pvParameters;
    uint32_t current_ppg = 0;

    while (1) {
        if(max30101_read_fifo(max_handle, &current_ppg) == ESP_OK){
            
            if(xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
                ble_sensor_payload.ppg_green = current_ppg;
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read PPG: %lu", current_ppg);
            }

        }else {
            ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
        }
    vTaskDelay(pdMS_TO_TICKS(10));     // 1/50Hz = 0.020s
    }
}

void ble_update_task(void *pvParameters){
    while (1){
        // Only send if a phone is connected
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {

            // Is ble_sensor_payload free from producers?
            if(xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
                struct os_mbuf *om = ble_hs_mbuf_from_flat(&ble_sensor_payload, sizeof(ble_sensor_payload));
                if(om != NULL){
                    ble_gatts_notify_custom(conn_handle, sensor_chr_val_handle, om);
                }
                xSemaphoreGive(sensor_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1/2Hz = 0.5s    // 1/0,5s = 2Hz
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Initialize I2C-bus and I2C-sensors
    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);
    i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);
    
    ESP_LOGI(TAG, "All sensors initialized.");
    
    // Semaphore's job is to prevent multiple processes to read/write to ble_payload struct
    // at the same time.
    sensor_data_mutex = xSemaphoreCreateMutex();

    init_ble_server();

    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, bmi_handle, 10, NULL, 1);        // High-speed IMU Task (100Hz)
    xTaskCreatePinnedToCore(temp_task, "temp_task", 2048, tmp_handle, 2, NULL, 1);       // Slow Temperature Task (1Hz)
    xTaskCreatePinnedToCore(ppg_task, "ppg_task", 4096, max_handle, 9, NULL, 1);         // Heart Rate Task (50Hz)
    xTaskCreatePinnedToCore(ble_update_task, "ble_update_task", 4096, NULL, 4, NULL, 1); // Send struct via BLE (100 ms)

    sensor_data_mutex = xSemaphoreCreateMutex();
    
    return;
}

