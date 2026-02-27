#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_random.h" // For generating random numbers
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "S3_BEACON";
static uint8_t ble_addr_type;



void update_advertisement_data() {
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)"S3_BEACON";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    uint8_t packet[] = {0xFF, 0xFF, 0x00};
    packet[2] = (uint8_t) (esp_random() % 100); //(Random 0-99)
    fields.mfg_data = packet;
    fields.mfg_data_len = sizeof(packet);

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting fields; rc=%d", rc);
        return;
    }else {
        ESP_LOGI(TAG, "Updated Beacon value: %x", packet);
    }
}
void ble_update_task(void *pvParameters) {
    while(1){
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_advertisement_data();
    }
}

// This function sets up what the beacon "shouts"
void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    // 1. Set Flags: General Discovery, No Bluetooth Classic
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    fields.name = (uint8_t *)"S3_BEACON";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    /*
    uint8_t custom_values[] = {0xDE, 0xAD, 0xBE, 0xEF};
    fields.mfg_data = custom_values;
    fields.mfg_data_len = sizeof(custom_values);*/

    // This wrapper handles the 4-arg function you found internally!
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting fields; rc=%d", rc);
        return;
    }

    // 4. Configure as a Broadcaster (NON-CONNECTABLE)
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // KEY: No connections allowed
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // Start shouting
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Beacon is live and broadcasting!");

    xTaskCreate(ble_update_task, "ble_update_task", 4096, NULL, 5, NULL);
}


void ble_on_sync(void) {
    // Automatically determine address type (Public vs Random)
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void ble_host_task(void *param) {
    nimble_port_run(); // NimBLE infinite loop
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(4000));

    // 1. Initialize NVS (Storage for BLE stack)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. Initialize NimBLE and GAP
    nimble_port_init();
    ble_svc_gap_init();
    
    // 3. Start the background BLE process
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
}