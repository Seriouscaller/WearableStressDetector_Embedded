#include "gatt.h"
#include "ble_server.h"

const ble_uuid128_t ble_sensor_svc_uuid;
const ble_uuid128_t ble_sensor_chr_a_uuid;
const ble_uuid128_t ble_sensor_chr_b_uuid;
const ble_uuid128_t ble_sensor_chr_c_uuid;
const struct ble_gatt_svc_def gatt_svcs[];

// Custom 128-bit UUIDs (Little Endian)
// Service ("folder")
// 00 00 ff 01 ab cd 12 34 ab cd 12 34 56 78 9a bc
const ble_uuid128_t ble_sensor_svc_uuid = BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab,
                                                           0x34, 0x12, 0xcd, 0xab, 0x01, 0xff, 0x00, 0x00);

// Characteristic ("part a")
// 00 00 ff 03 ab cd 12 34 ab cd 12 34 56 78 9a bc
const ble_uuid128_t ble_sensor_chr_a_uuid = BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab,
                                                             0x34, 0x12, 0xcd, 0xab, 0x03, 0xff, 0x00, 0x00);

// Characteristic ("part b")
// 00 00 ff 04 ab cd 12 34 cd cd 12 34 56 78 9a bc
const ble_uuid128_t ble_sensor_chr_b_uuid = BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab,
                                                             0x34, 0x12, 0xcd, 0xab, 0x04, 0xff, 0x00, 0x00);

// Characteristic ("part c")
// 00 00 ff 05 ab cd 12 34 cd cd 12 34 56 78 9a bc
const ble_uuid128_t ble_sensor_chr_c_uuid = BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab,
                                                             0x34, 0x12, 0xcd, 0xab, 0x05, 0xff, 0x00, 0x00);

// GATT Table Definition
// Creates a custom service ("folders"), with a custom sensor-
// characteristic ("files"). Assigns which callback-function to run
// when phone tries to read value (gets latest data).
// Assigns permission for phone to READ values from char, and the S3
// to push/notify when new data is available.
// Data pushed over BLE consist of complete_data_t which is split
// into 3 parts to fit into Max Transmissible Unit. Each part
// becomes a separate characteristic. The three parts are
// reassembled on the receiving side (python script).
const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &ble_sensor_svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){{.uuid = &ble_sensor_chr_a_uuid.u, // UUID ...03ff...   Part A
                                      .access_cb = sensor_read_cb,
                                      .flags = BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &ble_sensor_chr_a_val_handle},
                                     {.uuid = &ble_sensor_chr_b_uuid.u, // UUID ...04ff...   Part B
                                      .access_cb = sensor_read_cb,
                                      .flags = BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &ble_sensor_chr_b_val_handle},
                                     {.uuid = &ble_sensor_chr_c_uuid.u, // UUID ...05ff...   Part C
                                      .access_cb = sensor_read_cb,
                                      .flags = BLE_GATT_CHR_F_NOTIFY,
                                      .val_handle = &ble_sensor_chr_c_val_handle},
                                     {0}}},
    {0}};