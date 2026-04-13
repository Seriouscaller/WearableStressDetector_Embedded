// Storage for collecting sensor data to a ring buffer on PSRAM.
// Ring buffer for raw_data_t serves as the base to extract 30 second
// windows of data for feature extraction.

#include "board_config.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/ringbuf.h"
#include "shared_variables.h"
#include "types.h"

void check_spiffs_status(char *partition_name);

static const char *TAG = "STORAGE";

esp_err_t init_raw_data_ring_buffer(RingbufHandle_t *ring_buffer)
{
    // Allocating memory in PSRAM
    *ring_buffer = xRingbufferCreateWithCaps(RING_BUF_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);

    if (*ring_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to create Ring Buffer in PSRAM!");
        return ESP_ERR_NO_MEM;
    } else {
        ESP_LOGI(TAG, "Ring Buffer created: %d bytes in PSRAM", RING_BUF_SIZE);
        return ESP_OK;
    }
}

esp_err_t init_storage_transfer_learning(void)
{
    esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                  .partition_label = PARTITION_NAME,
                                  .max_files = 5,
                                  .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init_storage_training_data - Failed to create SPIFFS filesystem!");
        return ret;
    }

    check_spiffs_status(PARTITION_NAME);

    return ESP_OK;
}

void check_spiffs_status(char *partition_name)
{
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(partition_name, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}