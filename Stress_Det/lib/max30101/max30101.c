// MAX30101 PPG Optical blood volume pulse sensor

#include "max30101.h"
#include "board_config.h"
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

// Adds sensor to i2c
// Configure device
esp_err_t max30101_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *max_handle)
{

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

    // Green LED set to 25.4 mA
    // [0x00 - 0xFF (51 mA)]
    // Can use both registers to drive Green Led. Double power.
    write_reg(*max_handle, MAX30101_REG_LED3_PA, MAX30101_LED_PA_25_4MA);
    write_reg(*max_handle, MAX30101_REG_LED4_PA, 0x00);

    // 1 sample averging
    write_reg(*max_handle, MAX30101_REG_FIFO_CONFIG, MAX30101_FIFO_AVG_1);

    // Sampling rate: 200Hz
    // ADC Resolution: 18-bit
    write_reg(*max_handle, MAX30101_REG_SPO2_CFG, MAX30101_SAMPLE_RATE_200_ADC_18B);

    // Enable Multi-LED Mode (0x07)
    // Is needed to enable green LED
    write_reg(*max_handle, MAX30101_REG_MODE_CFG, MAX30101_MODE_MULTI_LED);

    ESP_LOGI(TAG, "Initialized successfully");
    return ret;
}

// Reads 18-bits of PPG data from sensor. Converts from little-endian
// to big endian uint32_t for correct data representation.
esp_err_t max30101_read_fifo(i2c_master_dev_handle_t dev_handle, uint32_t *ppg_green)
{
    uint8_t buffer[3];
    uint8_t reg = MAX30101_REG_FIFO_DATA;

    // Receives 3 Bytes bits PPG data from sensor. Stored in buffer.
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 3, -1);

    // Shifts bytes into uint32_t format
    if (ret == ESP_OK) {
        *ppg_green = (uint32_t)((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) & 0x3FFFF;
    }
    return ret;
}