#include "ble_server.h"
#include "board_config.h"
#include "esp_log.h"
#include "gatt.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "shared_variables.h"
#include "types.h"

static const char *TAG = "BLE";
static uint8_t ble_addr_type;
extern uint16_t ble_conn_handle;
extern SemaphoreHandle_t ble_payload_mutex;
extern ble_payload_bulk_t ble_payload_bulk_a;
extern ble_payload_bulk_t ble_payload_bulk_b;
extern ble_payload_bulk_t ble_payload_bulk_c;
extern ble_payload_bulk_t ble_payload_bulk_d;
extern ble_payload_final_t ble_payload_final;
extern uint16_t ble_sensor_chr_a_val_handle;
extern uint16_t ble_sensor_chr_b_val_handle;
extern uint16_t ble_sensor_chr_c_val_handle;
extern uint16_t ble_sensor_chr_d_val_handle;
extern uint16_t ble_sensor_chr_e_val_handle;
extern const struct ble_gatt_svc_def gatt_svcs[];

static void ble_app_advertise(void);

// Callback when phone requests the characteristic value
int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // If the PC manually asks for data, give it the latest split part
    if (xSemaphoreTake(ble_payload_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (attr_h == ble_sensor_chr_a_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payload_bulk_a, sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_sensor_chr_b_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payload_bulk_b, sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_sensor_chr_c_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payload_bulk_c, sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_sensor_chr_d_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payload_bulk_d, sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_sensor_chr_e_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payload_final, sizeof(ble_payload_bulk_t));
        }

        xSemaphoreGive(ble_payload_mutex);
        return 0;
    } else {
        ESP_LOGW(TAG, "Sensor_read_cb - Failed to take semaphore! ");
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// Callback when phone requests the name of characteristics
int gatt_svr_dsc_access(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (arg != NULL) {
        return os_mbuf_append(ctxt->om, arg, strlen((char *)arg));
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// GAP Event Handler (nimBLE)
// Handles connection-events of the BLE. Restarts advertising if
// a connected device disconnects. Can also negotiate a new
// Maximum Transmission Unit (MTU) if more space is needed
// for the BLE packet.
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connected! Status: %d", event->connect.status);
        ble_conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected. Restarting Advertising...");
        ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated to %d bytes", event->mtu.value);
        break;
    }
    return 0;
}

// Starts the BLE advertisement of device. Connectable, discoverable.
// Device shows as "XIAO_S3" in mobile nRF-application
static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Discoverable + Bluetooth Classic not supported
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)"XIAO_S3";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    memset(&adv_params, 0, sizeof(adv_params));

    // Undirected connectable. Looking for any device to connect to.
    // General Discoverable. Keeps sending advertisement packets.
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// When the internal BLE layers have synced up. Advertisement starts.
// Makes sure host and controller are in sync, and ready to handle BLE
// commands.
static void ble_on_sync(void)
{
    // No preference between public and random ble address.
    ble_hs_id_infer_auto(0, &ble_addr_type);

    ble_app_advertise();
}

// FreeRTOS task to let BLE run as a asynchronous task.
static void ble_host_task(void *param)
{
    // Blocking task to keep BLE running
    nimble_port_run();
    // Clean up when task is shutdown
    nimble_port_freertos_deinit();
}

// Collects ble related functions that is needed for startup of BLE
// communications.
void init_ble_server(void)
{
    // Non-volatile storage setup
    // Requirement for BLE stack to store critical information
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
        ESP_LOGE(TAG, "Failed to init NVS!");
        return;
    }

    // NimBLE initialization functions
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Configures Generic Attribute Profile (GAT)
    // Checks how many Bytes are needed for the current sensor
    // values that we'll send.
    ble_gatts_count_cfg(gatt_svcs);

    // Registers the characteristics of our sensors to the BLE database
    // so that connecting device can see them.
    ble_gatts_add_svcs(gatt_svcs);

    // When radio is ready, advertisement starts.
    ble_hs_cfg.sync_cb = ble_on_sync;

    // Starting and running BLE on core 0
    xTaskCreatePinnedToCore(ble_host_task, "ble_host_task", 4096, NULL, 5, NULL, 0);
}
