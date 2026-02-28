#include "ble.h"
#include "i2c_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void ble_app_advertise(void);
void update_advertisement_data();
static void ble_on_sync(void);
static void ble_host_task(void *param);
void ble_update_task(void *pvParameters);

static const char *TAG = "BLE";
static const char* BLE_DEVICE_NAME = "S3";
static uint8_t ble_addr_type;

// 13 Bytes. Company(2) + GSR(2) + Temp(1) + PPG(2) + acc(6)
// Advertisement packet size-limit 31-byte
typedef struct {
    uint16_t company_id; // 2B
    uint16_t gsr;        // 2B
    uint16_t temp_raw;   // 2B
    uint32_t ppg_green;  // 4B
    int16_t acc_x;       // 2B
    int16_t acc_y;       // 2B
    int16_t acc_z;       // 2B
    int16_t gyr_x;       // 2B
} __attribute__((packed)) sensor_payload_t;

static sensor_payload_t global_payload = {.company_id = 0xFFFF };

static void ble_on_sync(void) {
    // Automatically determine address type (Public vs Random)
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

// This function sets up what the beacon "shouts"
void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    // 1. Set Flags: General Discovery, No Bluetooth Classic
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    fields.name = (uint8_t*)BLE_DEVICE_NAME;
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;
    
    uint8_t custom_values[] = {0xFF, 0xFF, 0xFF, 0xFF};
    fields.mfg_data = custom_values;
    fields.mfg_data_len = sizeof(custom_values);
    
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
    
    // BLE Update interval 160 * 0.625 ms = 100 ms
    // Y * 0.625 = x
    // ms / 0.625 = reg_value
    adv_params.itvl_min = 90;
    adv_params.itvl_max = 90;

    // Start shouting
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Beacon is live and broadcasting!");
    update_advertisement_data();

    xTaskCreate(ble_update_task, "ble_update_task", 4096, NULL, 5, NULL);
}

static void ble_host_task(void *param) {
    nimble_port_run(); // NimBLE infinite loop
}

void init_ble(void) {
    //Initialize NVS (Storage for BLE stack)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

void ble_update_sensor_data(uint16_t gsr, float temp, uint32_t ppg, int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x) {
    global_payload.gsr = gsr;
    global_payload.temp_raw = (uint16_t)(temp * 100);
    global_payload.ppg_green = ppg;
    global_payload.acc_x = acc_x;
    global_payload.acc_y = acc_y;
    global_payload.acc_z = acc_z;
    global_payload.gyr_x = gyr_x;
}

void update_advertisement_data() {
    struct ble_hs_adv_fields fields;
    int rc;

    // FLAGS 3 Bytes (Length + type + flags)
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // NAME 4 Bytes
    // (Length + Type + Name (2B))
    fields.name = (uint8_t*)BLE_DEVICE_NAME;
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    // Manufacturer Specific Data
    // Company ID 2 Bytes +  Manufacturer data 
    fields.mfg_data = (uint8_t*)&global_payload;
    fields.mfg_data_len = sizeof(global_payload);

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting fields; rc=%d", rc);
        return;
    }else {
        ESP_LOGI(TAG, "Updated Beacon value\n GSR: %u PPG: %lu Temp: %u Acc_x: %d Acc_y: %d Acc_z: %d Gyr_x: %d",
        global_payload.gsr, global_payload.ppg_green, global_payload.temp_raw, global_payload.acc_x, 
        global_payload.acc_y, global_payload.acc_z, global_payload.gyr_x);
    }
}

void ble_update_task(void *pvParameters) {
    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));
        update_advertisement_data();
    }
}
