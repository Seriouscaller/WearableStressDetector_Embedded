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
#include "types.h"

static const char *TAG = "BLE";
static uint8_t ble_addr_type;
extern uint16_t conn_handle;
extern sensor_data_t ble_sensor_payload;
extern SemaphoreHandle_t sensor_data_mutex;

void ble_app_advertise(void);

// Callback when phone reads the characteristic
// Checks if bluetooth data struct is not being written to, if so, adds runtime to struct,
// packs data from ble_sensor_payload into outgoing ble-messagebuffer.
int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ble_sensor_payload.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
        os_mbuf_append(ctxt->om, &ble_sensor_payload, sizeof(ble_sensor_payload));
        xSemaphoreGive(sensor_data_mutex);
    }
    return 0;
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
        conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected. Restarting Advertising...");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
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
void ble_app_advertise(void)
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
void ble_on_sync(void)
{
    // No preference between public and random ble address.
    ble_hs_id_infer_auto(0, &ble_addr_type);

    ble_app_advertise();
}

// FreeRTOS task to let BLE run as a asynchronous task.
void ble_host_task(void *param)
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
