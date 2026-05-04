#include "spi_common.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "SPI";

/**
 * @brief  Initializes the SPI Master Bus for the CJMCU 6701 GSR sensor.
 *
 * Configures the GPIO pins and DMA channel for SPI communication. This function
 * uses the SPI2_HOST (FSPI) controller, which is the standard peripheral bus
 * for the ESP32-S3.
 *
 * @return
 *      - ESP_OK:   SPI bus initialized successfully.
 *      - ESP_FAIL: Initialization failed (typically due to invalid GPIOs or
 *                  bus already in use).
 */
esp_err_t init_spi(void)
{
    // Bus Configuration
    // Assigning MISO, MOSI, and SCLK pins as defined in board_config.h
    spi_bus_config_t buscfg = {.miso_io_num = SPI_NUM_MISO,
                               .mosi_io_num = SPI_NUM_MOSI,
                               .sclk_io_num = SPI_NUM_CLK,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1,
                               .max_transfer_sz = 32};

    /**
     * 2. Bus Initialization
     * Using SPI_DMA_CH_AUTO allows the IDF to allocate a DMA channel,
     * ensuring non-blocking transfers which is crucial for 200Hz sampling.
     */
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus. Err: %d", ret);
        return ESP_FAIL;
    }

    return ret;
};
