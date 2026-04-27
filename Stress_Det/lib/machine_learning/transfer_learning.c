#include "transfer_learning.h"
#include "esp_log.h"
#include "types.h"
#include <stdio.h>
/*

0. Open the file

Phase 1:
1. Validate data. Are there too much empty values?
2. Do we have enough different data? Phase variety

Phase 2:
3. Normalize the data using the mix and max of the collected data

Phase 3:
4. Iterative tuning
 - Find BMU of data
 - Nudge the neighborhood
Learning rate starts very tiny, and decays over time

Phase 4:
5. Benchmark the model
 - Calculate Quantization Error, and compare old model with new
 - If error is larger, the model is worse
6. Replace old model
7. Clear the binary data file on the device

Phase 5:
8. Check for custom map in memory. Use new map if found,
   else go back to default.
9. Start inference

*/
#define FILESIZE 100000

typedef struct {
    uint32_t file_size;
    char filename[30];
    uint32_t num_of_samples;
    som_input_transfer_learning_t training_data[FILESIZE];
} learning_t;

const char *TAG = "TRANSFER_LEARNING";

esp_err_t open_training_data(learning_t *trainer)
{
    uint32_t len = trainer->file_size;
    FILE *ptr;
    ptr = fopen("spiffs/log.bin", "rb");

    if (ptr == NULL) {
        ESP_LOGE(TAG, "open_training_data, Failed to open file!");
        return ESP_FAIL;
    }

    som_input_transfer_learning_t sample;
    while (fread(&sample, sizeof(som_input_transfer_learning_t), 1, ptr) == 1) {
        trainer->training_data[trainer->num_of_samples] = sample;

        if (trainer->num_of_samples >= len) {
            ESP_LOGE(TAG, "open_training_data, Buffer full!");
            fclose(ptr);
            return ESP_FAIL;
        }
        trainer->num_of_samples++;
    }
    fclose(ptr);

    return ESP_OK;
}

void display_data(learning_t *trainer)
{
    for (int i = 0; i < trainer->num_of_samples; i++) {
        som_input_transfer_learning_t smpl = trainer->training_data[i];
        ESP_LOGI(TAG, "[%u] hr:%.2f rmssd:%.2f sc_ph:%.4f sc_rr:%.4f", i, smpl.features.hr,
                 smpl.features.hrv_rmssd, smpl.features.sc_ph, smpl.features.sc_rr);
    }
}

// How much of data is empty?
void validate_data()
{
}

void start_transfer_learning(void)
{
    learning_t transfer_learning = {.file_size = FILESIZE, .filename = "spiffs/bin.log", .num_of_samples = 0};
    esp_err_t res = open_training_data(&transfer_learning);
}