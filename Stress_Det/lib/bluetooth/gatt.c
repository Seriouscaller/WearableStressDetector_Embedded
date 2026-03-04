#include "gatt.h"
#include "ble_conn.h" 
#include "esp_timer.h"

// 128-bit UUIDs (Little Endian)
static const ble_uuid128_t sensor_svc_uuid = 
    BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab, 0x34, 0x12, 0xcd, 0xab, 0x01, 0xff, 0x00, 0x00);

static const ble_uuid128_t sensor_chr_uuid = 
    BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab, 0x34, 0x12, 0xcd, 0xab, 0x02, 0xff, 0x00, 0x00);

uint16_t sensor_chr_val_handle;

// Read Callback: When the phone requests data manually
static int sensor_read_cb(uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // Access the data globally defined in ble_conn.c
    extern ble_sensor_payload_t ble_payload; 
    os_mbuf_append(ctxt->om, &ble_payload, sizeof(ble_sensor_payload_t));
    return 0;
}

// GATT Table Definition
const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &sensor_svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = &sensor_chr_uuid.u,
          .access_cb = sensor_read_cb,
          .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &sensor_chr_val_handle},
         {0}}},
    {0}};