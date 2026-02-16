#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "max30101.h"
#include "tmp117.h"
#include "i2c_common.h"

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

i2c_master_dev_handle_t start_i2c(){
    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);

    i2c_master_dev_handle_t max_handle;
    ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));


    return max_handle;
}

void display_raw_ppg_value(i2c_master_dev_handle_t max_handle, max30101_data_t max_data){
    if (max30101_read_fifo(max_handle, &max_data) == ESP_OK) {
        printf(">PPG_Green_Raw:%lu\n", max_data.green_raw);
    } else {
        ESP_LOGW(TAG, "I2C Read Failed or FIFO Empty");
    }
}

void display_temperature(i2c_master_dev_handle_t tmp_handle){
    float temperature = 0.0f;
    esp_err_t ret = tmp117_read_temp(tmp_handle, &temperature);
    if(ret == ESP_OK){
        printf(">Temperature:%.2f\n", temperature);
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000)); 

    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);

    i2c_master_dev_handle_t max_handle = add_max30101_i2c(bus_handle);
    i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);

    max30101_data_t max_data;

    while(1) {
        //display_raw_ppg_value(max_handle, max_data);
        display_temperature(tmp_handle);
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
