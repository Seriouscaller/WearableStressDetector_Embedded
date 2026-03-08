#include "gsr.h"
#include "spi_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

#define GSR_R_FIXED 100000.0f;
#define GSR_V_REF   3.3f;
#define SPI_NUM_CS    2

static const char *TAG = "GSR";

esp_err_t gsr_sensor_init(spi_device_handle_t *gsr_handle) {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, 
        .mode = 0,                         
        .spics_io_num = SPI_NUM_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
    };
    
    return spi_bus_add_device(SPI2_HOST, &devcfg, gsr_handle);
}

esp_err_t gsr_sensor_read_raw(spi_device_handle_t handle, uint16_t* raw){
    uint8_t rx_data[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_transmit(handle, &t) != ESP_OK;

    if(ret == ESP_OK){
        *raw = ((rx_data[0] & 0x1F) << 7 | (rx_data[1] >> 1));
    }

    return ret;
}