#include "spi_common.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Pin definitions
#define SPI_NUM_MISO  8     // XIAO D9
#define SPI_NUM_CLK   7     // XIAO D8
#define SPI_NUM_MOSI -1

esp_err_t init_spi(void){
    // Init SPI-bus
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_NUM_MISO,
        .mosi_io_num = SPI_NUM_MOSI,
        .sclk_io_num = SPI_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    // Using the 3rd SPI-bus on the S3. Standard for peripherals
    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
};
