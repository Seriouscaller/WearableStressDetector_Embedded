#include "ble_commands.h"
#include "gatt.h"
#include <stdio.h>

#define BLE_CMD_START_SAMPLING 0x01
#define BLE_CMD_STOP_SAMPLING 0x02
#define BLE_CMD_GET_BATTERY 0x03
#define BLE_CMD_REBOOT 0x04

static const char *TAG = "BLE_cmd";
extern uint16_t ble_command_chr_val_handle;
extern uint8_t ble_received_command;

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

    // Copy the data from ble message buffer to our global variable
    os_mbuf_copydata(ctxt->om, 0, sizeof(uint8_t), &ble_received_command);

    // Is the received data the correct size?
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != sizeof(uint8_t)) {
        ESP_LOGE(TAG, "Incorrect length of received command!");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    ESP_LOGI(TAG, "Received: Dec %u Hex 0x%02X", ble_received_command, ble_received_command);
    return 0;
}