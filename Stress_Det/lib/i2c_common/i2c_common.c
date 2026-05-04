#include "i2c_common.h"
#include "board_config.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C";

#define I2C_MASTER_GLITCH_FILTER_LEN 7

/**
 * @brief Initializes the I2C Master Bus for the sensor suite.
 *
 * Configures the I2C0 peripheral on the XIAO S3. This bus is shared by the
 * MAX30101 (PPG), BMI260 (IMU), and TMP117 (Temp). The configuration includes
 * a glitch filter to suppress bus noise and enables internal pull-ups to
 * maintain signal integrity.
 *
 * @param[out] bus_handle Pointer to the handle that will store the created I2C bus.
 *
 * @return
 *      - ESP_OK:   I2C bus initialized successfully.
 *      - ESP_FAIL: Peripheral allocation or configuration failed.
 */
esp_err_t init_i2c(i2c_master_bus_handle_t *bus_handle)
{
    /**
     * I2C Master Bus Configuration
     * ----------------------------
     * .i2c_port: Uses the hardware I2C0 controller.
     * .sda_io_num / .scl_io_num: Mapped to XIAO D4 and D5.
     * .glitch_ignore_cnt: Set to 7 (standard) to filter spikes on the SDA/SCL lines.
     */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_MASTER_GLITCH_FILTER_LEN,
        .flags.enable_internal_pullup = true,
    };

    // Initialize I2C master bus
    esp_err_t ret = i2c_new_master_bus(&bus_config, bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus. Err: %d", ret);
        return ESP_FAIL;
    }

    return ret;
};

/**
 * @brief Writes a single byte of data to a specific register of an I2C device.
 *
 * This is a synchronous helper function that encapsulates the i2c_master_transmit
 * logic. It prepends the register address to the data byte, which is the standard
 * format expected by most sensors (MAX30101, BMI260, etc.).
 *
 * @param[in] handle The handle of the target I2C device (e.g., ppg_handle).
 * @param[in] reg    The internal register address to write to.
 * @param[in] data   The byte of data to write into the register.
 *
 * @return
 *      - ESP_OK: Transmission successful.
 *      - ESP_ERR_TIMEOUT: Operation timed out.
 *      - ESP_ERR_INVALID_STATE: I2C bus not initialized or busy.
 */
esp_err_t write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(handle, buf, sizeof(buf), -1);
}

/**
 * @brief Reads a single byte of data from a specific I2C register.
 *
 * Uses the ESP-IDF v5.x master-bus API to perform a 'Write-then-Read'
 * transaction. This is the standard procedure for querying sensor
 * configurations or status registers.
 *
 * @param[in]  handle The I2C device handle (e.g., ppg_handle, imu_handle).
 * @param[in]  reg    The internal register address to query.
 * @param[out] data   Pointer to the buffer where the read byte will be stored.
 *
 * @return
 *      - ESP_OK:   Read operation successful.
 *      - ESP_FAIL: Communication error or device NACK.
 *
 * @note The timeout is set to -1 (infinite), which is safe for initialization
 *       but should be replaced with a tick-based timeout in high-frequency loops.
 */
esp_err_t read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data)
{
    // i2c_master_transmit_receive performs the write-then-read sequence
    // It sends the 'reg' address and then reads 1 byte into 'data'
    return i2c_master_transmit_receive(handle, &reg, 1, data, 1, -1);
}