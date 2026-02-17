#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "max30101.h"
#include "tmp117.h"
#include "gsr.h"
#include "i2c_common.h"
#include "spi_common.h"
#include "driver/spi_master.h"

static const char *TAG = "MAIN_APP";

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

spi_device_handle_t add_gsr_spi() {
    spi_device_handle_t gsr_handle;
    ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));
    return gsr_handle;
}

void display_raw_ppg_value(i2c_master_dev_handle_t max_handle, max30101_data_t max_data){
    if (max30101_read_fifo(max_handle, &max_data) == ESP_OK) {
        printf(">PPG_Green_Raw:%lu\n", max_data.green_raw);
    } else {
        ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
    }
}

void display_temperature(i2c_master_dev_handle_t tmp_handle){
    float temperature = 0.0f;
    esp_err_t ret = tmp117_read_temp(tmp_handle, &temperature);
    if(ret == ESP_OK){
        printf(">Temperature:%.2f\n", temperature);
    } else {
        ESP_LOGW(TAG, "Failed to read TMP117 sensor.");
    }
}

void display_raw_gsr(spi_device_handle_t gsr_handle){

    uint16_t raw_gsr = 0;

    if (gsr_sensor_read_raw(gsr_handle, &raw_gsr) == ESP_OK) {
        float voltage = (raw_gsr / 4095.0f) * GSR_V_REF;
        printf(">Raw GSR:%u\n", raw_gsr);
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

    /*
    // I2C Sensors (PPG & Temp)
    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);
    i2c_master_dev_handle_t max_handle = add_max30101_i2c(bus_handle);
    i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);
    max30101_data_t max_data;*/

    // SPI Sensor (GSR)
    ESP_ERROR_CHECK(init_spi());
    spi_device_handle_t gsr_handle = add_gsr_spi();

    ESP_LOGI(TAG, "All sensors initialized.");

    while(1) {
        //display_raw_ppg_value(max_handle, max_data);
        //display_temperature(tmp_handle);
        display_raw_gsr(gsr_handle);
        vTaskDelay(200 / portTICK_PERIOD_MS); // Faster sampling (5Hz)
    }
}
