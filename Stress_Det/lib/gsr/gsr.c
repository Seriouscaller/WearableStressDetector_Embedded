// GSR Galvanic Skin Response Sensor

#include "gsr.h"
#include "board_config.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_common.h"

static const char *TAG = "GSR";
extern bool show_gsr_debugging;

#define GSR_SPI_MODE 0
#define GSR_SPI_QUEUE_SIZE 1
#define GSR_SPI_TICK_COUNT 16

// Adds GSR to SPI-bus
esp_err_t gsr_sensor_init(spi_device_handle_t *gsr_handle)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQ_HZ,
        .mode = GSR_SPI_MODE,
        .spics_io_num = SPI_NUM_CS,
        .queue_size = GSR_SPI_QUEUE_SIZE,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
    };

    return spi_bus_add_device(SPI2_HOST, &devcfg, gsr_handle);
}

// Receives 2B of GSR data via SPI
esp_err_t gsr_sensor_read_raw(spi_device_handle_t handle, uint16_t *raw)
{
    // Setup
    uint8_t rx_data_buffer[2] = {0};
    spi_transaction_t t = {
        .length = GSR_SPI_TICK_COUNT,
        .rx_buffer = rx_data_buffer,
    };

    esp_err_t ret = spi_device_transmit(handle, &t);

    if (show_gsr_debugging)
        ESP_LOGI("GSR_DEBUG", "Byte0: 0x%02X, Byte1: 0x%02X", rx_data_buffer[0], rx_data_buffer[1]);

    // (rx_data[0] & 0x1F) Keeps last 5 bits of first Byte
    // (rx_data[1] >> 1) Discards trailing bit
    // (rx_data[0] & 0x1F) << 7  Makes room for rest of data
    if (ret == ESP_OK) {
        *raw = (uint16_t)((rx_data_buffer[0] & 0x1F) << 7 | (rx_data_buffer[1] >> 1));
    }

    return ret;
}