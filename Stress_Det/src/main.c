#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "max30101.h"
#include "tmp117.h"
#include "gsr.h"
#include "bmi160.h"
#include "ble.h"
#include "i2c_common.h"
#include "spi_common.h"
#include "driver/spi_master.h"

typedef struct {
    uint32_t ppg_green;
    float temperature_c;
    bmi160_data_t imu;
    uint16_t gsr_raw;
} sensor_data_t;

static const char *TAG = "MAIN_APP";
static uint8_t ble_addr_type;

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
        float voltage = (*raw_gsr / 4095.0f) * GSR_V_REF;
        printf(">Raw GSR:%u\n", *raw_gsr);
    } else {
        ESP_LOGW(TAG, "Failed to read GSR sensor.");
    }

    /*
    if(voltage > 0.05){
        float resistance = GSR_R_FIXED * ((GSR_V_REF / voltage) - 1.0f);
        float conductance = (1.0f / resistance) * 1000000.0f;

        printf(">Voltage:%.2f\n", voltage);
        printf(">GSR_uS:%.2f\n", conductance);
        printf(">Resistance:%.2f\n", resistance);

    }*/
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    
    // Initialize I2C-bus and I2C-sensors
    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);
    i2c_master_dev_handle_t max_handle = add_max30101_i2c(bus_handle);
    i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);
    i2c_master_dev_handle_t bmi_handle = add_bmi160_i2c(bus_handle);
    
    // Initialize SPI-bus and SPI-sensor
    ESP_ERROR_CHECK(init_spi());
    spi_device_handle_t gsr_handle = add_gsr_spi();
    
    sensor_data_t sensor_data = {0};
    
    ESP_LOGI(TAG, "All sensors initialized.");
    
    // 1. Initialize NVS (Storage for BLE stack)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    
    while(1) {
        display_raw_ppg(max_handle, &sensor_data.ppg_green);
        display_temperature(tmp_handle, &sensor_data.temperature_c);
        display_raw_imu(bmi_handle, &sensor_data.imu);
        display_raw_gsr(gsr_handle, &sensor_data.gsr_raw);

        ble_update_sensor_data(
            sensor_data.gsr_raw,
            sensor_data.temperature_c,
            sensor_data.ppg_green,
            sensor_data.imu.acc_x,
            sensor_data.imu.acc_y,
            sensor_data.imu.acc_z,
            sensor_data.imu.gyr_x
        );

        ESP_LOGD(TAG, "Pushed new sensor data to BLE buffer");
            
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


