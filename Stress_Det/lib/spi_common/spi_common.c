#include "spi_common.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

// Sets up master SPI-bus. Assigns pins.
esp_err_t init_spi(void)
{
    spi_bus_config_t buscfg = {.miso_io_num = SPI_NUM_MISO,
                               .mosi_io_num = SPI_NUM_MOSI,
                               .sclk_io_num = SPI_NUM_CLK,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1,
                               .max_transfer_sz = 32};

    // Using the 3rd SPI-bus on the S3. Standard for peripherals
    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
};
