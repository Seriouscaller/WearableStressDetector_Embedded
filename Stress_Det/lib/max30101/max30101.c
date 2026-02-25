#include "max30101.h"
#include "i2c_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX30101";

// --- MAX30102 Definitions ---
#define MAX30101_ADDR               0x57
#define MAX30101_REG_INT_ENABLE_1   0x02
#define MAX30101_REG_FIFO_WR_PTR    0x04
#define MAX30101_REG_FIFO_RD_PTR    0x06
#define MAX30101_REG_FIFO_DATA      0x07
#define MAX30101_REG_FIFO_CONFIG    0x08
#define MAX30101_REG_MODE_CFG       0x09
#define MAX30101_REG_MULTI_LED_1    0x11
#define MAX30101_REG_MULTI_LED_2    0x12
#define MAX30101_REG_SPO2_CFG       0x0A
#define MAX30101_REG_LED1_PA        0x0C // Red LED
#define MAX30101_REG_LED2_PA        0x0D // IR LED
#define MAX30101_REG_LED3_PA        0x0E // Green LED
#define MAX30101_REG_LED4_PA        0x0F // Green LED #2

esp_err_t max30101_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* max_handle){

    // Add device with address 0x57
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30101_ADDR,
        .scl_speed_hz = 400000,
    };

    // Adding sensor to i2c-bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, max_handle);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to add device to i2c bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // First comms with sensor
    // Reset sensor
    ret = write_reg(*max_handle, MAX30101_REG_MODE_CFG, 0x40);     
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to communicate with device: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Clear FIFO pointers
    write_reg(*max_handle, MAX30101_REG_FIFO_WR_PTR, 0x00);
    write_reg(*max_handle, MAX30101_REG_FIFO_RD_PTR, 0x00);

    // Slot Configuration: Define the sequence of LED firing
    // Only interested in green led for wrist heartrate detection
    // 0x21: Slot 1 = Green (1), Slot 2 = Disabled (0)
    write_reg(*max_handle, MAX30101_REG_MULTI_LED_1, 0x03);
    // 0x00: Slot 0 = Disabled (0), Slot 4 = Disabled (0)
    write_reg(*max_handle, MAX30101_REG_MULTI_LED_2, 0x00);

    // Green LED set to 25.4 mA
    // [0x00 - 0xFF (51 mA)]
    // Can use both registers to drive Green Led. Double power.
    write_reg(*max_handle, MAX30101_REG_LED3_PA, 0x7F);
    write_reg(*max_handle, MAX30101_REG_LED4_PA, 0x00);
    
    // FIFO & ADC Setup
    // Sampling rate: 50Hz
    // ADC Resolution: 18-bit
    write_reg(*max_handle, MAX30101_REG_FIFO_CONFIG, 0x40); 
    write_reg(*max_handle, MAX30101_REG_SPO2_CFG, 0x43);

    // Enable Multi-LED Mode (0x07)
    // Needed to enable green LED
    write_reg(*max_handle, MAX30101_REG_MODE_CFG, 0x07);
    
    ESP_LOGI(TAG, "Initialized successfully");
    return ret;

    /* TODO: Enums for hex-values! */
}

esp_err_t max30101_read_fifo(i2c_master_dev_handle_t dev_handle, uint32_t* ppg_green){
    uint8_t buffer[3];
    uint8_t reg = MAX30101_REG_FIFO_DATA;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 3, -1);

    if(ret == ESP_OK){
        *ppg_green = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) & 0x3FFFF;
    }
    return ret;
}