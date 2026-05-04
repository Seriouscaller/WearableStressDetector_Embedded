// GSR Galvanic Skin Response Sensor

#include "gsr.h"
#include "board_config.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_common.h"
#include "types.h"

static const char *TAG = "GSR";
extern device_control_t device_config;

#define GSR_SPI_MODE 0
#define GSR_SPI_QUEUE_SIZE 1
#define GSR_SPI_TICK_COUNT 16

/**
 * @brief Registers the GSR sensor (CJMCU 6701) on the SPI bus.
 *
 * This function adds the GSR sensor to the previously initialized SPI bus.
 * It configures the SPI clock speed and mode specific to the CJMCU 6701's
 * analog-to-digital converter.
 *
 * @param[out] gsr_handle Pointer to the handle that will represent this SPI device.
 *
 * @return
 *      - ESP_OK: Device added to the SPI bus successfully.
 *      - ESP_ERR_INVALID_ARG: Handle or configuration is null.
 *      - ESP_ERR_NO_MEM: Out of memory for device allocation.
 */
esp_err_t gsr_sensor_init(spi_device_handle_t *gsr_handle)
{
    /**
     * SPI Device Configuration
     * ------------------------
     * .mode: Determines the CPOL/CPHA clock polarity/phase.
     * .spics_io_num: The GPIO used for Chip Select (defined in board_config.h).
     * .queue_size: Number of transactions that can be queued (usually 1-7).
     */
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

/**
 * @brief  Reads the raw ADC value from the GSR sensor via SPI.
 *
 * Performs a synchronous SPI transaction to retrieve two bytes from the
 * CJMCU 6701. The function then extracts a 12-bit value by masking
 * out the leading 3 bits (status/null) and the trailing bit.
 *
 * @param[in]  handle SPI device handle for the GSR sensor.
 * @param[out] raw    Pointer to store the reconstructed 12-bit raw ADC value.
 *
 * @return
 *      - ESP_OK:   Data successfully read and parsed.
 *      - ESP_ERR_*: SPI transmission error.
 */
esp_err_t gsr_sensor_read_raw(spi_device_handle_t handle, uint16_t *raw)
{
    // Setup
    uint8_t rx_data_buffer[2] = {0};

    /**
     * SPI Transaction Configuration
     * .length: Total number of bits to transmit/receive (usually 16).
     * .rx_buffer: Pointer to the 2-byte destination array.
     */
    spi_transaction_t t = {
        .length = GSR_SPI_TICK_COUNT,
        .rx_buffer = rx_data_buffer,
    };

    esp_err_t ret = spi_device_transmit(handle, &t);

    if (device_config.show_gsr_debugging)
        ESP_LOGI("GSR_DEBUG", "Byte0: 0x%02X, Byte1: 0x%02X", rx_data_buffer[0], rx_data_buffer[1]);

    /**
     * Bit-Masking & Reconstruction
     *
     * Typical 12-bit ADC SPI Frame: [XX 0 B11 B10 B9 B8 B7 B6] [B5 B4 B3 B2 B1 B0 X X]
     * - (buffer[0] & 0x1F): Masks out the first 3 bits (Null/Status).
     * - (<< 7): Shifts the high 5 bits into position.
     * - (buffer[1] >> 1): Discards the trailing bit and aligns the low 7 bits.
     */
    if (ret == ESP_OK) {
        *raw = (uint16_t)((rx_data_buffer[0] & 0x1F) << 7 | (rx_data_buffer[1] >> 1));
    }

    return ret;
}