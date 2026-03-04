#include "ble_conn.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "gatt.h"
#include "types.h"

static const char *TAG = "BLE_CONN";
static uint8_t ble_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
ble_sensor_payload_t ble_payload = { .company_id = 0x02E5 };

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    os_mbuf_append(ctxt->om, &ble_payload, sizeof(ble_sensor_payload_t));
    return 0;
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"XIAO_S3_BIO";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connected! Status: %d", event->connect.status);
            conn_handle = event->connect.conn_handle;
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected. Restarting Advertising...");
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise();
            break;
    }
    return 0;
}

static void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

static void ble_host_task(void *param) {
    nimble_port_run();
}

void ble_conn_set_data(const app_sensor_data_t *data) {

    ble_payload.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    ble_payload.gsr = data->gsr_raw;
    ble_payload.ppg_green = data->ppg_green;
    ble_payload.temp_raw = (uint16_t)(data->temperature_c * 100);

    ble_payload.acc_x = data->imu.accel_x;
    ble_payload.acc_y = data->imu.accel_y;
    ble_payload.acc_z = data->imu.accel_z;

    ble_payload.gyr_x = data->imu.gyro_x;
    ble_payload.gyr_y = data->imu.gyro_y;
    ble_payload.gyr_z = data->imu.gyro_z;

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&ble_payload, sizeof(ble_sensor_payload_t));
        ble_gatts_notify_custom(conn_handle, sensor_chr_val_handle, om);
    }
}

void init_ble_conn(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE Connection Stack Initialized");
}