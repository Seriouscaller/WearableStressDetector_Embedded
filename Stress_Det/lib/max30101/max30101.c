// MAX30101 PPG Optical blood volume pulse sensor

#include "max30101.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_common.h"

static const char *TAG = "MAX30101";

#define MAX30101_ADDR 0x57
#define MAX30101_REG_INT_ENABLE_1 0x02
#define MAX30101_REG_FIFO_WR_PTR 0x04
#define MAX30101_REG_FIFO_RD_PTR 0x06
#define MAX30101_REG_FIFO_DATA 0x07
#define MAX30101_REG_FIFO_CONFIG 0x08
#define MAX30101_REG_MODE_CFG 0x09
#define MAX30101_REG_MULTI_LED_1 0x11
#define MAX30101_REG_MULTI_LED_2 0x12
#define MAX30101_REG_SPO2_CFG 0x0A
#define MAX30101_REG_LED1_PA 0x0C // Red LED
#define MAX30101_REG_LED2_PA 0x0D // IR LED
#define MAX30101_REG_LED3_PA 0x0E // Green LED
#define MAX30101_REG_LED4_PA 0x0F // Green LED #2

#define MAX30101_MODE_RESET 0x40
#define MAX30101_PTR_RESET 0x00
#define MAX30101_SLOT_1_GREEN 0x03
#define MAX30101_SLOT_3_4_DIS 0x00
#define MAX30101_LED_PA_25_4MA 0x7F
#define MAX30101_FIFO_AVG_1 0x00
#define MAX30101_FIFO_AVG_4 0x40
#define MAX30101_SAMPLE_RATE_100_ADC_18B 0x43
#define MAX30101_SAMPLE_RATE_200_ADC_18B 0x0B
#define MAX30101_MODE_MULTI_LED 0x07
#define MAX30101_FIFO_DATA_MASK 0x3FFFF

#define BOOSTPMP_5V_ENABLE_PIN 2

/**
 * @brief  Initializes the MAX30101 for Green-only PPG heart rate detection.
 *
 * Sets up the 5V boost power supply, configures the I2C device on the bus,
 * and puts the sensor into Multi-LED mode. This specific configuration is
 * optimized for wrist-based heart rate monitoring by utilizing both green LED
 * channels to maximize signal penetration.
 *
 * @param[in]  bus_handle I2C master bus handle for the XIAO S3.
 * @param[out] max_handle Pointer to the handle for the MAX30101 device.
 *
 * @return esp_err_t ESP_OK on success, or an error code on communication failure.
 */
esp_err_t max30101_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *max_handle)
{
    gpio_reset_pin(BOOSTPMP_5V_ENABLE_PIN);
    gpio_set_direction(BOOSTPMP_5V_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BOOSTPMP_5V_ENABLE_PIN, 1);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30101_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Adding sensor to i2c-bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, max_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to i2c bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // First comms with sensor
    // Reset sensor
    ret = write_reg(*max_handle, MAX30101_REG_MODE_CFG, MAX30101_MODE_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with device: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Clear FIFO pointers
    write_reg(*max_handle, MAX30101_REG_FIFO_WR_PTR, MAX30101_PTR_RESET);
    write_reg(*max_handle, MAX30101_REG_FIFO_RD_PTR, MAX30101_PTR_RESET);

    // Slot Configuration: Define the sequence of LED firing
    // Only interested in green led for wrist heartrate detection
    write_reg(*max_handle, MAX30101_REG_MULTI_LED_1, MAX30101_SLOT_1_GREEN);
    write_reg(*max_handle, MAX30101_REG_MULTI_LED_2, MAX30101_SLOT_3_4_DIS);

    /**
     * LED Drive Current & Power Management
     * Pulse Amplitude 0x7F corresponds to ~25.4 mA.
     */
    write_reg(*max_handle, MAX30101_REG_LED3_PA, 0x7F);
    write_reg(*max_handle, MAX30101_REG_LED4_PA, 0x7F);

    // Signal Quality Settings
    // 2-sample averaging helps mitigate high-frequency I2C bus noise.
    write_reg(*max_handle, MAX30101_REG_FIFO_CONFIG, 0x20);

    /**
     * ADC Configuration:
     * 200 Hz sampling rate at 18-bit resolution (411µs pulse width).
     */
    write_reg(*max_handle, MAX30101_REG_SPO2_CFG, 0x2F);

    // Enable Multi-LED Mode (0x07)
    // Is needed to enable green LED
    write_reg(*max_handle, MAX30101_REG_MODE_CFG, MAX30101_MODE_MULTI_LED);

    ESP_LOGI(TAG, "Initialized successfully");
    return ret;
}

/**
 * @brief  Reads a single PPG sample from the MAX30101 FIFO.
 *
 * In Multi-LED mode, the FIFO is organized by the slots defined in the
 * Multi-LED Control registers. Since only Slot 1 (Green) is active,
 * a single 3-byte read retrieves the most recent Green LED intensity.
 *
 * @param[in]  dev_handle I2C device handle for the MAX30101.
 * @param[out] ppg_green  Pointer to store the 18-bit filtered PPG value.
 *
 * @return
 *      - ESP_OK: Data read and bit-shifted successfully.
 *      - ESP_FAIL: I2C communication error.
 *
 * @note Masking with 0x3FFFF removes the upper 6 bits of the 24-bit
 *       read, which are padding/reserved bits in 18-bit ADC mode.
 */
esp_err_t max30101_read_fifo(i2c_master_dev_handle_t dev_handle, uint32_t *ppg_green)
{
    uint8_t buffer[3];
    uint8_t reg = MAX30101_REG_FIFO_DATA;

    /**
     * i2c_master_transmit_receive:
     * Sends the FIFO_DATA register address and then clocks out
     * 3 bytes (24 bits) of data in a single I2C transaction.
     */
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 3, -1);

    if (ret == ESP_OK) {
        /**
         * Reconstruct the 24-bit value:
         * Byte 0: [X][X][B17][B16][B15][B14][B13][B12] (High Byte)
         * Byte 1: [B11][B10][B09][B08][B07][B06][B05][B04]
         * Byte 2: [B03][B02][B01][B00][X][X][X][X] (Low Byte)
         *
         * The & 0x3FFFF mask extracts the 18 bits of ADC data.
         */
        *ppg_green = (uint32_t)((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) & 0x3FFFF;
    }
    return ret;
}

/**
 * @brief  Calculates the number of unread samples available in the sensor FIFO.
 *
 * The MAX30101 uses a circular buffer. This function reads the Write Pointer
 * (controlled by the sensor) and the Read Pointer (controlled by the MCU)
 * to determine the data backlog.
 *
 * @param[in]  dev_handle I2C device handle for the MAX30101.
 * @param[out] count      Number of available samples (0 to 32).
 *
 * @return esp_err_t ESP_OK on successful I2C communication.
 */
esp_err_t max30101_get_fifo_count(i2c_master_dev_handle_t dev_handle, uint8_t *count)
{
    uint8_t write_ptr = 0;
    uint8_t read_ptr = 0;
    esp_err_t err;

    // 1. Read the Write Pointer (where the sensor is currently writing)
    err = read_reg(dev_handle, MAX30101_REG_FIFO_WR_PTR, &write_ptr);
    if (err != ESP_OK)
        return err;

    // 2. Read the Read Pointer (where our code last stopped reading)
    err = read_reg(dev_handle, MAX30101_REG_FIFO_RD_PTR, &read_ptr);

    if (err != ESP_OK)
        return err;

    // 3. Calculate the difference
    // The FIFO is a circular buffer of 32 slots.
    if (write_ptr >= read_ptr) {
        *count = write_ptr - read_ptr;
    } else {
        // Handle wrap-around case
        *count = (32 - read_ptr) + write_ptr;
    }

    return ESP_OK;
}