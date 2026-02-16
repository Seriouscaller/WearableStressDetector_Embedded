#include "tmp117.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMP117";

// --- TMP117 Definitions ---
#define TMP117_ADDR               0x48
#define TMP117_REG_DEVICE_ID      0x0F
#define TMP117_REG_TEMP_RESULT    0x00
#define TMP117_REG_CONFIG         0x01
#define TMP117_RESOLUTION         0.0078125f

static esp_err_t write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data){
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(handle, buf, sizeof(buf), -1);
}

esp_err_t tmp117_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* tmp_handle){

    // Add device with address 0x48
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP117_ADDR,
        .scl_speed_hz = 400000,
    };

    // Adding sensor to i2c-bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, tmp_handle);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to add device to i2c bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Initialized successfully");
    return ret;
}

esp_err_t tmp117_read_temp(i2c_master_dev_handle_t tmp_handle, float* temperature){
    uint8_t reg_addr = TMP117_REG_TEMP_RESULT;
    uint8_t data[2] = {0};

    // Tell sensor what register we like to read from. Returns data.
    esp_err_t ret = i2c_master_transmit_receive(tmp_handle, &reg_addr, 1, data, 2, -1);

    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to read temperature register: %s", esp_err_to_name);
        return ret;
    }

    // TMP117 store temp as Little-Endian. Convert to big endian.
    int16_t raw_temp =(int16_t) ((data[0] << 8) | data[1]);
    *temperature = (float) raw_temp * TMP117_RESOLUTION;

    return ret;
}