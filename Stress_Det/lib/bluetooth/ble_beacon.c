#include "ble_beacon.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BLE_CTRL";
static const char* BLE_DEVICE_NAME = "S3";
static uint8_t ble_addr_type;

typedef struct {
    uint16_t company_id; 
    uint16_t gsr;        
    uint16_t temp_raw;   
    uint32_t ppg_green;  
    int16_t acc_x;       
    int16_t acc_y;       
    int16_t acc_z;       
    int16_t gyr_x;       
} __attribute__((packed)) sensor_payload_t;

static sensor_payload_t global_payload = {.company_id = 0xFFFF };

static void ble_on_sync(void);
static void ble_host_task(void *param);
static void ble_update_task(void *pvParameters);
static void ble_app_advertise(void);
static void update_advertisement_data(void);

void init_ble_beacon(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE Stack Initialized");
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

static void update_advertisement_data(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t*)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    fields.mfg_data = (uint8_t*)&global_payload;
    fields.mfg_data_len = sizeof(global_payload);

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Adv fields error: %d", rc);
    }
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    // 56.25ms interval (90 * 0.625)
    adv_params.itvl_min = 90;
    adv_params.itvl_max = 90;

    update_advertisement_data();

    int rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Beacon Broadcasting...");
        xTaskCreate(ble_update_task, "ble_upd", 4096, NULL, 5, NULL);
    }
}

static void ble_update_task(void *pvParameters) {
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(50)); 
        update_advertisement_data();
    }
}

static void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

static void ble_host_task(void *param) {
    nimble_port_run();
}