#include "ble_commands.h"
#include "esp_spiffs.h"
#include "gatt.h"
#include "shared_variables.h"
#include "storage.h"
#include <stdio.h>

#define BLE_CMD_START_SAMPLING 0x01
#define BLE_CMD_STOP_SAMPLING 0x02
#define BLE_CMD_SET_EXP_PHASE 0x03
#define BLE_CMD_CLEAR_SPIFFS 0x55
#define BLE_CMD_REBOOT 0x99

#define SIZE_PACKET_WITH_PAYLOAD 2

static const char *TAG = "BLE_cmd";
extern uint16_t ble_command_chr_val_handle;
extern uint8_t ble_received_packet[];
extern volatile bool is_sampling_active;
extern volatile uint8_t current_experiment_phase;
extern SemaphoreHandle_t experiment_phase_mutex;

// Callback when device receives command over ble
int control_write_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Was a write operation received?
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ESP_LOGE(TAG, "Incorrect operation code!");
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Did we receive the expected characteristic?
    if (attr_h != ble_command_chr_val_handle) {
        ESP_LOGE(TAG, "Unexpected characteristics received!");
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Is the received data the correct size, if so
    // Copy the data from ble message buffer to our global variables
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    memset(ble_received_packet, 0, SIZE_PACKET_WITH_PAYLOAD * sizeof(uint8_t));

    // Receved a 1 Byte command without payload
    if (len == sizeof(uint8_t)) {
        os_mbuf_copydata(ctxt->om, 0, sizeof(uint8_t), ble_received_packet);
        ESP_LOGI(TAG, "Received: Hex 0x%02X", ble_received_packet[0]);

        // Receved a 2 Byte command with payload
    } else if (len == (SIZE_PACKET_WITH_PAYLOAD * sizeof(uint8_t))) {
        os_mbuf_copydata(ctxt->om, 0, SIZE_PACKET_WITH_PAYLOAD * sizeof(uint8_t), ble_received_packet);
        ESP_LOGI(TAG, "Received: Hex 0x%02X Payload: 0x%02X", ble_received_packet[0], ble_received_packet[1]);

    } else {
        ESP_LOGE(TAG, "Incorrect length of received command!");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (ble_received_packet[0] == BLE_CMD_SET_EXP_PHASE) {
        // BLE_CMD_SET_EXP_PHASE command must have a payload
        if (len < SIZE_PACKET_WITH_PAYLOAD) {
            ESP_LOGI(TAG, "Set Phase failed: Payload missing!");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        // Store experimentphase in global variable to be able to log it
        if (xSemaphoreTake(experiment_phase_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_experiment_phase = ble_received_packet[1];
            xSemaphoreGive(experiment_phase_mutex);

            ESP_LOGI(TAG, "Experimentphase set: %d", ble_received_packet[1]);
        } else {
            ESP_LOGW(TAG, "control_write_cb - Failed to take semaphore. Exp. Phase not retrieved!");
        }

    } else if (ble_received_packet[0] == BLE_CMD_START_SAMPLING) {
        ESP_LOGI(TAG, "Sampling started!");
        is_sampling_active = true;
    } else if (ble_received_packet[0] == BLE_CMD_STOP_SAMPLING) {
        ESP_LOGI(TAG, "Sampling halted!");
        is_sampling_active = false;
    } else if (ble_received_packet[0] == BLE_CMD_CLEAR_SPIFFS) {
        esp_err_t res = esp_spiffs_format(PARTITION_NAME);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "SPIFFS Formatting!");
            check_spiffs_status(PARTITION_NAME);
        } else {
            ESP_LOGW(TAG, "SPIFFS Formatting failed!");
        }
    } else if (ble_received_packet[0] == BLE_CMD_REBOOT) {
        ESP_LOGI(TAG, "Device rebooting!");
        esp_restart();
    }

    return 0;
}