#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_ble_mesh_defs.h"

/* ================== Vendor Model IDs ================== */
#define DF_MESH_VND_COMPANY_ID          0x8888
#define DF_MESH_VND_MODEL_ID_SERVER     0x0000

/* Tạo 3-byte vendor opcode từ 1-byte opcode + Company ID */
#define DF_VND_OPCODE(n)                ESP_BLE_MESH_MODEL_OP_3((n), DF_MESH_VND_COMPANY_ID)

/* ================== Vendor Opcodes ================== */
/* OpIdx 0x01 = SET   (Gateway → Node: gửi lệnh/thông tin fire)  */
/* OpIdx 0x02 = GET   (Gateway → Node: truy vấn trạng thái)      */
/* OpIdx 0x03 = STATUS (Node → Gateway: phản hồi)                 */
/* OpIdx 0x11 = NODE_STATUS (Node tự broadcast trạng thái)        */
#define DF_MESH_VND_OP_SET      ESP_BLE_MESH_MODEL_OP_3(0x01, DF_MESH_VND_COMPANY_ID)
#define DF_MESH_VND_OP_GET      ESP_BLE_MESH_MODEL_OP_3(0x02, DF_MESH_VND_COMPANY_ID)
#define DF_MESH_VND_OP_STATUS   ESP_BLE_MESH_MODEL_OP_3(0x03, DF_MESH_VND_COMPANY_ID)
#define DF_MESH_VND_OP_NODE_ST  ESP_BLE_MESH_MODEL_OP_3(0x11, DF_MESH_VND_COMPANY_ID)

/* ================== Fire Alarm Commands ================== */
#define DF_CMD_FIRE_ALARM_NORMAL        0x00
#define DF_CMD_FIRE_ALARM_TRIGGER       0x01
#define DF_CMD_FIRE_ALARM_MUTE          0x02

/* ================== Cấu trúc payload Vendor ================== */
/**
 * @brief Payload chuẩn cho OpIdx 0x01 (SET) và 0x03 (STATUS)
 *        Kích thước: sizeof(st_vendor_custom_data_t) = 24 byte
 */
typedef struct __attribute__((packed)) {
    uint8_t  u8Cmd;             /* DF_CMD_FIRE_ALARM_*             */
    uint16_t u16PacketId;       /* Số thứ tự gói (rolling counter) */
    int16_t  s16Temperature;    /* Nhiệt độ × 10, ví dụ 350 = 35.0°C */
    uint16_t u16SmokeVal;       /* Nồng độ khói (PPM)              */
    uint8_t  u8AlarmState;      /* 0=Bình thường, 1=Báo cháy       */
    char     acDeviceTag[16];   /* Tên thiết bị (null-terminated)  */
} st_vendor_custom_data_t;

/**
 * @brief Payload OpIdx 0x11 – NODE_STATUS (thiết bị nút / đầu khói tự phát)
 *        Tối thiểu 2 byte: [type][status][MAC x6 – optional]
 */
typedef struct __attribute__((packed)) {
    uint8_t u8DevType;      /* 0x01=Nút bấm, 0x02=Đầu khói */
    uint8_t u8Status;       /* 0x01=Bình thường, 0x02=Báo động/Kích hoạt, 0x03=Hết khói/Kết thúc */
    uint8_t au8Mac[6];      /* Địa chỉ MAC (optional, present nếu length >= 8) */
} st_vendor_node_status_t;
