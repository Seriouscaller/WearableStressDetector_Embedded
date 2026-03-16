// General purpose I2C scanner for Esp32-S3.
// Detects devices on bus and prints addresses.

#include "board_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "I2C_SCAN";

void app_main(void)
{
    // 1. Safety Delay: Wait for USB Serial to connect
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "--- I2C Scanner Starting ---");

    // 2. Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    // Initialize the driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    // 3. Scan Loop
    while (1) {
        ESP_LOGI(TAG, "Scanning...");
        int devices_found = 0;

        for (int i = 1; i < 127; i++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            // (i << 1) shifts address to make room for R/W bit
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);

            // Attempt to send command. If we get ACK (ESP_OK), a device is there.
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, ">> Found device at address: 0x%02X", i);
                devices_found++;
            }
        }

        if (devices_found == 0) {
            ESP_LOGW(TAG, "No devices found.");
            ESP_LOGW(TAG, "Check: 1. Pull-up resistors (4.7k). 2. Swapped SDA/SCL.");
        } else {
            ESP_LOGI(TAG, "Scan complete. Found %d device(s).", devices_found);
        }

        ESP_LOGI(TAG, "-----------------------");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Scan every 2 seconds
    }
}