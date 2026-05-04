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
extern ble_payload_bulk_t ble_payloads_bulk[];
extern ble_payload_final_t ble_payload_final;
extern ble_sensor_handles_t ble_val_handles;
extern const struct ble_gatt_svc_def gatt_svcs[];

static void ble_app_advertise(void);

/**
 * @brief  Callback function for BLE Read operations on sensor data characteristics.
 *
 * This function serves as the interface for manual data retrieval. When a BLE central
 * reads one of the sensor characteristics (A-I), this function retrieves the
 * corresponding data segment from the global bulk storage. It uses a mutex to ensure
 * data integrity, preventing the BLE stack from reading a buffer while it is being
 * updated by the sensor sampling tasks.
 *
 * @param[in] conn_h Connection handle.
 * @param[in] attr_h Attribute handle of the characteristic being read.
 * @param[in] ctxt   Pointer to the GATT access context (where data is appended).
 * @param[in] arg    User-defined argument (unused).
 *
 * @return
 *      - 0: Success.
 *      - BLE_ATT_ERR_UNLIKELY: Failed to acquire mutex or internal error.
 */
int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg)
{

    if (xSemaphoreTake(ble_payload_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (attr_h == ble_val_handles.ble_sensor_chr_a_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[0], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_b_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[1], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_c_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[2], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_d_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[3], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_e_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[4], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_f_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[5], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_g_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[6], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_h_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payloads_bulk[7], sizeof(ble_payload_bulk_t));
        } else if (attr_h == ble_val_handles.ble_sensor_chr_i_val_handle) {
            os_mbuf_append(ctxt->om, &ble_payload_final, sizeof(ble_payload_final_t));
        }

        xSemaphoreGive(ble_payload_mutex);
        return 0;
    } else {
        ESP_LOGW(TAG, "Sensor_read_cb - Failed to take semaphore! ");
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/**
 * @brief  Callback for accessing BLE GATT Characteristic Descriptors.
 *
 * This is a generic callback used to provide metadata for various characteristics.
 * When a descriptor is defined in the GATT server (e.g., a User Description),
 * the static string passed via the 'arg' parameter in the service definition
 * is appended to the mbuf for transmission to the central device.
 *
 * @param[in] conn_h Connection handle identifying the active BLE link.
 * @param[in] attr_h Attribute handle of the descriptor being accessed.
 * @param[in] ctxt   Pointer to the GATT access context containing the mbuf chain.
 * @param[in] arg    Pointer to the null-terminated string to be sent (passed
 *                   from the service definition).
 *
 * @return
 *      - 0: Success (data successfully appended to mbuf).
 *      - BLE_ATT_ERR_UNLIKELY: Failure, typically if 'arg' is NULL.
 */
int gatt_svr_dsc_access(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (arg != NULL) {
        return os_mbuf_append(ctxt->om, arg, strlen((char *)arg));
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/**
 * @brief  Callback for BLE Generic Access Profile (GAP) events.
 *
 * This function handles the lifecycle of a BLE connection. It captures the
 * connection handle upon a successful link, restarts the advertising process
 * if the connection is dropped, and monitors MTU negotiations to optimize
 * throughput for sensor data transfers.
 *
 * @param[in] event Pointer to the GAP event structure.
 * @param[in] arg   User-defined argument (unused).
 *
 * @return
 *      - 0: Success.
 *      - Non-zero: Error code (refer to NimBLE return codes).
 *
 * @note Re-advertising on disconnect is essential for headless wearable operation.
 */
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

/**
 * @brief  Configures and starts the BLE advertising process.
 *
 * Sets up the advertising payload, including the device name ("XIAO_S3") and
 * required GAP flags. It then starts advertising in general discoverable and
 * undirected connectable mode, allowing any central device to initiate a
 * connection.
 *
 * @note The device uses 'BLE_HS_FOREVER', meaning it will broadcast indefinitely
 *       until a peer connects or the stack is manually stopped.
 *
 * @see ble_gap_adv_set_fields
 * @see ble_gap_adv_start
 */
static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    /**
     * @details Set Advertising Flags:
     * - BLE_HS_ADV_F_DISC_GEN: General Discoverable mode.
     * - BLE_HS_ADV_F_BREDR_UNSUP: Indicates the device does not support
     *   Bluetooth Classic (EDR), which is required for LE-only devices.
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)"XIAO_S3";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    memset(&adv_params, 0, sizeof(adv_params));

    /**
     * @details Connection and Discovery Modes:
     * - BLE_GAP_CONN_MODE_UND: Undirected connectable mode (any central can connect).
     * - BLE_GAP_DISC_MODE_GEN: General discovery mode (no timeout on discovery).
     */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

/**
 * @brief  Callback triggered when the BLE host and controller are synchronized.
 *
 * This function is mandatory for NimBLE applications. It ensures that the BLE
 * stack is fully initialized before any GAP or GATT operations are performed.
 * Once synced, it automatically determines the best address type to use and
 * initiates the advertising process.
 *
 * @note This is called by the NimBLE host task, not the main application loop.
 *
 * @see ble_hs_id_infer_auto
 * @see ble_app_advertise
 */
static void ble_on_sync(void)
{
    /**
     * Determine the address type automatically.
     * 0: No preference for public address.
     * &ble_addr_type: Pointer where the determined address type (public or random)
     * is stored for use in advertising.
     */
    ble_hs_id_infer_auto(0, &ble_addr_type);

    ble_app_advertise();
}

/**
 * @brief  FreeRTOS task responsible for running the NimBLE stack.
 *
 * This task calls 'nimble_port_run()', which is a blocking function that
 * executes the NimBLE host stack event loop. It handles all background
 * processing for GAP, GATT, and L2CAP. If the loop ever returns (e.g., during
 * a stack shutdown), it performs the necessary de-initialization to free
 * FreeRTOS resources.
 *
 * @param[in] param Pointer to task parameters (typically NULL).
 *
 * @note This task should be pinned to a core (usually Core 0) if the other
 *       core is heavily utilized by high-frequency sensor sampling.
 */
static void ble_host_task(void *param)
{
    // Blocking task to keep BLE running
    nimble_port_run();
    // Clean up when task is shutdown
    nimble_port_freertos_deinit();
}

/**
 * @brief  Initializes the BLE stack, GATT server, and starts the host task.
 *
 * This function follows the standard startup sequence for a NimBLE server:
 * 1. Prepares NVS flash (required for BLE security/stack state).
 * 2. Initializes the NimBLE port and core services (GAP/GATT).
 * 3. Calculates and allocates memory for the GATT service table.
 * 4. Sets the synchronization callback to trigger advertising.
 * 5. Spawns the blocking BLE host task on Core 0.
 *
 * @note This should be called once during the system power-up sequence,
 *       typically after hardware peripherals (ADC/SPI/I2C) are ready.
 *
 * @see nvs_flash_init
 * @see nimble_port_init
 * @see ble_on_sync
 */
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
