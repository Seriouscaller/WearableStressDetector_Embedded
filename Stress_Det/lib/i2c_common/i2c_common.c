#include "i2c_common.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h" 

static const char *TAG = "I2C";

#define I2C_MASTER_SDA_IO 5     // Physical Pin D4 (S3) #define I2C_MASTER_SDA_IO 5
#define I2C_MASTER_SCL_IO 6     // Physical Pin D5 (S3) #define I2C_MASTER_SCL_IO 6

void init_i2c(i2c_master_bus_handle_t* bus_handle){
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
};

esp_err_t write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data){
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(handle, buf, sizeof(buf), -1);
}