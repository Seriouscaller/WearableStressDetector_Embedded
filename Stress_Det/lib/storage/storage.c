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

/**
 * @brief  Initializes a thread-safe Ring Buffer in PSRAM for raw sensor data.
 *
 * Creates a No-Split ring buffer to store 'raw_data_t' structures. By using
 * PSRAM (MALLOC_CAP_SPIRAM), the system can buffer large windows of 200Hz
 * data without exhausting internal SRAM.
 *
 * @param[out] ring_buffer Pointer to the handle where the created buffer is stored.
 *
 * @return
 *      - ESP_OK:         Buffer successfully allocated in PSRAM.
 *      - ESP_ERR_NO_MEM: Failed to allocate memory (PSRAM full or not initialized).
 *
 * @note RINGBUF_TYPE_NOSPLIT is chosen to ensure each 'raw_data_t' item is
 *       retrieved as a contiguous memory block, simplifying pointer arithmetic
 *       in the consumer task.
 */
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

/**
 * @brief Initializes the SPIFFS flash filesystem for persistent model storage.
 *
 * This function mounts the SPIFFS partition used to store transfer learning
 * snapshots and the adapted SOM weights. It ensures that user-specific
 * stress baselines persist across deep sleep cycles or battery swaps.
 *
 * @return
 *      - ESP_OK: SPIFFS mounted successfully and ready for R/W.
 *      - ESP_FAIL: Mounting failed or partition not found in the partition table.
 *      - ESP_ERR_NOT_FOUND: The partition label does not exist.
 *
 * @note If mounting fails, 'format_if_mount_failed' is set to true, which will
 *       reformat the partition. This is useful for first-time setup but will
 *       erase existing user profiles.
 */
esp_err_t init_storage_transfer_learning(void)
{
    /**
     * SPIFFS Configuration:
     * - base_path: The VFS mount point (e.g., fopen("/spiffs/model.bin", "w"))
     * - partition_label: Matches the name in 'partitions.csv' (e.g., "storage")
     * - max_files: 5 is sufficient for model weights, metadata, and log files.
     */
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

/**
 * @brief  Logs the storage utilization of a specific SPIFFS partition.
 *
 * Fetches the total capacity and currently occupied space of the flash filesystem.
 * This is used to ensure that transfer learning data or experiment logs do not
 * exceed the allocated partition size, which would prevent further model updates.
 *
 * @param[in] partition_name The label of the partition to check (matches partitions.csv).
 */
void check_spiffs_status(char *partition_name)
{
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(partition_name, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}