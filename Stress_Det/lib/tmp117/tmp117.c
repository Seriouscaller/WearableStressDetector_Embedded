// TMP117 Temperature sensor

#include "tmp117.h"
#include "board_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMP117";

// --- TMP117 Definitions ---
#define TMP117_ADDR 0x48
#define TMP117_REG_DEVICE_ID 0x0F
#define TMP117_REG_TEMP_RESULT 0x00
#define TMP117_REG_CONFIG 0x01
#define TMP117_RESOLUTION 0.0078125f

/**
 * @brief  Registers the TMP117 high-precision temperature sensor to the I2C bus.
 *
 * Uses the ESP-IDF v5.x i2c_master driver to allocate a device handle. The TMP117
 * provides 16-bit resolution (0.0078°C per LSB) and is used here to monitor
 * skin temperature as a secondary stress indicator.
 *
 * @param[in]  bus_handle  Handle to the initialized I2C master bus.
 * @param[out] tmp_handle  Pointer to the handle that will represent this device.
 *
 * @return
 *      - ESP_OK: Device added successfully.
 *      - ESP_ERR_NO_MEM: Out of memory while adding device.
 *      - ESP_ERR_INVALID_ARG: Invalid bus handle or configuration.
 */
esp_err_t tmp117_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *tmp_handle)
{

    // Configure device with address 0x48
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP117_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Add sensor to i2c-bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, tmp_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to i2c bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Initialized successfully");
    return ret;
}

/**
 * @brief Reads the ambient/skin temperature from the TMP117.
 *
 * Performs a 16-bit register read and converts the 2's complement raw value
 * into degrees Celsius. The TMP117 uses a resolution of 0.0078125°C per LSB.
 *
 * @param[in]  tmp_handle  The I2C device handle for the TMP117.
 * @param[out] temperature Pointer to a float where the Celsius value will be stored.
 *
 * @return
 *      - ESP_OK: Temperature read and converted successfully.
 *      - ESP_ERR_TIMEOUT: I2C bus was busy or sensor did not respond.
 *      - Other ESP_ERR codes from the i2c_master_transmit_receive call.
 */
esp_err_t tmp117_read_temp(i2c_master_dev_handle_t tmp_handle, float *temperature)
{
    uint8_t reg_addr = TMP117_REG_TEMP_RESULT;
    uint8_t data[2] = {0};

    // Tell sensor what register we like to read from, and how
    // many Bytes. Sensor returns data.
    esp_err_t ret = i2c_master_transmit_receive(tmp_handle, &reg_addr, 1, data, 2, -1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature register: %s", esp_err_to_name);
        return ret;
    }

    // TMP117 store temp as Little-Endian. Convert to big endian.
    int16_t raw_temp = (int16_t)((data[0] << 8) | data[1]);
    *temperature = (float)raw_temp * TMP117_RESOLUTION;

    // TMP117 can return negative temperatures. For our use case, we set any negative value to 0.
    if (*temperature < 0) {
        *temperature = 0;
    }

    return ret;
}