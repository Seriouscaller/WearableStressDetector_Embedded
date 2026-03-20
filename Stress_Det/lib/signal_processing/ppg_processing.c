/*
Sliding-window buffer
Sliding window buffer continuesly adds ppg data. This is a circular buffer, which when reached the end,
starts overwriting the old data.

Snapshot buffer
When the SWB accumulated 30 seconds of data, the snapshot buffer function
makes a copy every second to a static array.
*/

#include "board_config.h"
#include "esp_log.h"
#include "types.h"
#include <stdio.h>

static void print_snapshot_summary(uint32_t *buffer, size_t len);

extern psram_ppg_ring_buffer_t ppg_sliding_window;
static const char *TAG = "PPG_PROC";
extern TaskHandle_t xPpgProcessingTaskHandle;
extern uint32_t processing_buffer; // 30 seconds of data approx 12 KB in RAM

void add_sample(psram_ppg_ring_buffer_t *buffer, uint32_t sample)
{
    if (xSemaphoreTake(buffer->lock, pdMS_TO_TICKS(5)) == pdTRUE) {
        buffer->data[buffer->head] = sample;
        buffer->head = (buffer->head + 1) % RING_BUF_CAPACITY;

        if (buffer->count < RING_BUF_CAPACITY) {
            buffer->count++;
        }
        xSemaphoreGive(buffer->lock);

        // Is there enough data for a snapshot?
        if (buffer->count >= SNAPSHOT_LEN && (buffer->head % PPG_SAMPLE_RATE == 0)) {
            // Notify other task that one sample was added to ppg sliding window buffer
            if (xPpgProcessingTaskHandle != NULL) {
                xTaskNotifyGive(xPpgProcessingTaskHandle);
                ESP_LOGI(TAG, "PPG-proc task notified!");
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to take semaphore lock");
    }
}

// Initializes a PPG PSRAM ringbuffer
esp_err_t init_ppg_psram_buffer(psram_ppg_ring_buffer_t *ppg_buffer, uint32_t samples_count)
{
    size_t buffer_size = samples_count * sizeof(uint32_t);

    // Allocate the massive block in Octal PSRAM
    ppg_buffer->data = (uint32_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);

    if (ppg_buffer->data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate log buffer!");
        return ESP_FAIL;
    }

    // Consider filling the buffer with 0s. (memset)

    // Initialize buffer variables
    ppg_buffer->head = 0;
    ppg_buffer->count = 0;
    ppg_buffer->lock = xSemaphoreCreateMutex(); // Semaphore to protect buffer access

    if (ppg_buffer->lock == NULL) {
        ESP_LOGE(TAG, "Failed to create buffer lock!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Allocated PPG psram buffer");
    return ESP_OK;
}

esp_err_t get_ppg_snapshot(psram_ppg_ring_buffer_t *ppg_buffer, uint32_t *dest_array)
{
    // Check if arguments are correct and initialized
    if (ppg_buffer == NULL || ppg_buffer->data == NULL || dest_array == NULL) {
        ESP_LOGE(TAG, "PPG buffer or destination array is NULL!");
        return ESP_ERR_INVALID_ARG;
    }

    // Did take semaphore succeed?
    if (xSemaphoreTake(ppg_buffer->lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take semaphore of ppg_buffer");
        return ESP_ERR_TIMEOUT;
    }

    // Do we have enough data in ppg buffer for a snapshot?
    if (ppg_buffer->count < SNAPSHOT_LEN) {
        ESP_LOGE(TAG, "Not enough data for 30 sec snapshot");
        xSemaphoreGive(ppg_buffer->lock);
        return ESP_ERR_INVALID_STATE;
    }

    // Calulate starting position of our 30 sec window
    // If start_idx turns negative, the window has wrapped around
    // the end of the buffer to the beginning
    int32_t start_idx = (int32_t)ppg_buffer->head - SNAPSHOT_LEN;
    if (start_idx < 0) {
        start_idx += RING_BUF_CAPACITY;
    }

    size_t first_part_len = RING_BUF_CAPACITY - start_idx;

    if (first_part_len >= SNAPSHOT_LEN) {
        // Case 1 if data is contiguous in buffer.
        memcpy(dest_array, &ppg_buffer->data[start_idx], SNAPSHOT_LEN * sizeof(uint32_t));
    } else {
        // Case 2 if head wrapped around to the beginning of array
        size_t second_part_len = SNAPSHOT_LEN - first_part_len;

        // Copy from start index to end of PSRAM buffer
        memcpy(dest_array, &ppg_buffer->data[start_idx], first_part_len * sizeof(uint32_t));

        // Copy remainder of window from beginning of PSRAM buffer
        memcpy(&dest_array[first_part_len], &ppg_buffer->data[0], second_part_len * sizeof(uint32_t));
    }

    xSemaphoreGive(ppg_buffer->lock);
    return ESP_OK;
}

void debug_ppg_buffer_status(psram_ppg_ring_buffer_t *rb)
{
    if (rb == NULL || rb->lock == NULL)
        return;

    if (xSemaphoreTake(rb->lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint32_t num_of_samples = rb->count;
        uint32_t head_of_buffer = rb->head;
        xSemaphoreGive(rb->lock);

        float fullness = ((float)num_of_samples / RING_BUF_CAPACITY) * 100.0f;

        // The "Tail" is the oldest data in the 90s buffer
        // If count < capacity, tail is 0. If full, tail is the head.
        uint32_t tail = (num_of_samples < RING_BUF_CAPACITY) ? 0 : head_of_buffer;

        ESP_LOGI("DEBUG_BUF", "--- PPG Buffer Status ---");
        ESP_LOGI("DEBUG_BUF", "Capacity:  %d samples (90s)", RING_BUF_CAPACITY);
        ESP_LOGI("DEBUG_BUF", "Current:   %lu samples", num_of_samples);
        ESP_LOGI("DEBUG_BUF", "Fullness:  %.2f%%", fullness);
        ESP_LOGI("DEBUG_BUF", "Head Pos:  %lu (Next Write)", head_of_buffer);
        ESP_LOGI("DEBUG_BUF", "Tail Pos:  %lu (Oldest Data)", tail);

        // Snapshot specific info
        if (num_of_samples >= SNAPSHOT_LEN) {
            int32_t snap_start = (int32_t)head_of_buffer - SNAPSHOT_LEN;
            if (snap_start < 0)
                snap_start += RING_BUF_CAPACITY;
            ESP_LOGI("DEBUG_BUF", "Last Snap: Starts at index %ld", snap_start);
        } else {
            ESP_LOGW("DEBUG_BUF", "Snapshot:  WAITING (%lu/%d samples)", num_of_samples, SNAPSHOT_LEN);
        }
        ESP_LOGI("DEBUG_BUF", "-------------------------");
    }
}

static void print_snapshot_summary(uint32_t *buffer, size_t len)
{
    ESP_LOGI("DATA_CHECK", "--- Snapshot Data (Every 100th sample) ---");
    for (int i = 0; i < len; i += 100) {
        printf("[%d]: %lu\n", i, buffer[i]);
    }
    ESP_LOGI("DATA_CHECK", "------------------------------------------");
}