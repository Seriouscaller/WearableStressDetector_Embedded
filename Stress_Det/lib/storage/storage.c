// Storage for collecting sensor data to a ring buffer on PSRAM.
// Ring buffer for raw_data_t serves as the base to extract 30 second
// windows of data for feature extraction.

#include "board_config.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "types.h"

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
