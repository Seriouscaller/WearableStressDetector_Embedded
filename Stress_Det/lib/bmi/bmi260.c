#include "bmi260.h"
#include "bmi260_config.h" // Contains the 8KB bmi260_config_file array
#include "board_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_common.h"

static esp_err_t upload_bmi260_config(i2c_master_dev_handle_t *dev_handle);

static const char *TAG = "BMI260";

#define BMI260_ADDR 0x68
#define BMI260_REG_CHIP_ID 0x00
#define BMI260_CHIP_ID_VAL 0x27
#define BMI260_REG_INTERNAL_STAT 0x21
#define BMI260_REG_INIT_CTRL 0x59
#define BMI260_REG_INIT_DATA 0x5E
#define BMI260_REG_PWR_CONF 0x7C
#define BMI260_REG_PWR_CTRL 0x7D
#define BMI260_REG_CMD 0x7E
#define BMI260_REG_DATA_ACC_X 0x0C
#define BMI260_REG_DATA_GYR_X 0x12
#define BMI260_REG_ACC_CONF 0x40
#define BMI260_REG_ACC_RANGE 0x41
#define BMI260_REG_GYR_CONF 0x42
#define BMI260_REG_GYR_RANGE 0x43
#define BMI260_INIT_FINALIZE 0x01
#define BMI260_INIT_SUCCESS 0x01
#define BMI260_CMD_SOFTRESET 0xB6

// Power Control Bits
#define BMI260_PWR_GYR_EN (1 << 1)
#define BMI260_PWR_ACC_EN (1 << 2)
#define BMI260_PWR_TEMP_EN (1 << 3)

esp_err_t bmi260_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI260_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle));

    // Soft Reset
    write_reg(*dev_handle, BMI260_REG_CMD, BMI260_CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Disable advanced power saving
    write_reg(*dev_handle, BMI260_REG_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Verify Chip ID (Expecting 0x27 for BMI260)
    uint8_t chip_id = 0;
    uint8_t reg = BMI260_REG_CHIP_ID;
    esp_err_t ret = i2c_master_transmit_receive(*dev_handle, &reg, 1, &chip_id, 1, -1);

    if (ret != ESP_OK || chip_id != BMI260_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Read failed or Wrong ID. Got: 0x%02X, Err: %d", chip_id, ret);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    upload_bmi260_config(dev_handle);
    vTaskDelay(pdMS_TO_TICKS(25)); // Wait for initialization

    // Check internal status
    uint8_t status = 0;
    reg = BMI260_REG_INTERNAL_STAT;
    i2c_master_transmit_receive(*dev_handle, &reg, 1, &status, 1, -1);
    if ((status & 0x0F) != BMI260_INIT_SUCCESS) {
        ESP_LOGE(TAG, "Initialization failed. Status: 0x%02X", status);
        return ESP_FAIL;
    }

    // Enable Accel and Gyro
    write_reg(*dev_handle, BMI260_REG_PWR_CTRL, BMI260_PWR_ACC_EN | BMI260_PWR_GYR_EN);

    ESP_LOGI(TAG, "BMI260 Initialized successfully");
    return ESP_OK;
}

static esp_err_t upload_bmi260_config(i2c_master_dev_handle_t *dev_handle)
{
    // Prepare for config upload
    write_reg(*dev_handle, BMI260_REG_INIT_CTRL, 0x00);

    // Upload Config File (Burst write)
    const size_t config_size = sizeof(bmi260_config_file);
    uint8_t *burst_buf = malloc(config_size + 1);

    if (burst_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for config upload");
        return ESP_FAIL;
    }

    burst_buf[0] = BMI260_REG_INIT_DATA;
    memcpy(&burst_buf[1], bmi260_config_file, config_size);

    ESP_LOGI(TAG, "Uploading configuration (%d bytes)...", config_size);
    esp_err_t ret = i2c_master_transmit(*dev_handle, burst_buf, config_size + 1, -1);
    free(burst_buf);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config upload failed. Err: %d", ret);
        return ret;
    }

    // Finalize config upload
    write_reg(*dev_handle, BMI260_REG_INIT_CTRL, BMI260_INIT_FINALIZE);
    return ESP_OK;
}

esp_err_t bmi260_read(i2c_master_dev_handle_t dev_handle, bmi_data_t *data)
{
    uint8_t reg = BMI260_REG_DATA_ACC_X; // Start at Accel registers
    uint8_t buf[12];
    uint8_t buf_size = sizeof(buf) / sizeof(buf[0]);

    // Read 12 bytes: 6 for Accel (0x0C-0x11), 6 for Gyro (0x12-0x17)
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, buf, buf_size, -1);

    if (ret == ESP_OK) {
        // REGISTER MAP ORDER: Accel is first
        data->acc_x = (int16_t)((buf[1] << 8) | buf[0]);
        data->acc_y = (int16_t)((buf[3] << 8) | buf[2]);
        data->acc_z = (int16_t)((buf[5] << 8) | buf[4]);

        data->gyr_x = (int16_t)((buf[7] << 8) | buf[6]);
        data->gyr_y = (int16_t)((buf[9] << 8) | buf[8]);
        data->gyr_z = (int16_t)((buf[11] << 8) | buf[10]);
    }
    return ret;
}
