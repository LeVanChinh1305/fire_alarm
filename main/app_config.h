#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"

/* ================== THÔNG TIN FIRMWARE ================== */
#define DF_FIRMWARE_VERSION             "1.1.0"
#define DF_FIRMWARE_NAME                "FIRE_ALARM_S3"
#define DF_DEVICE_TYPE                  4001
#define DF_HARDWARE_VERSION             "HW_V1.0"
#define DF_TARGET_CHIP                  CONFIG_IDF_TARGET

/* ================== CẤU HÌNH BLE MESH ================== */
#define DF_MESH_COMPANY_ID              0x02E5          /* Composition CID (Espressif)   */
#define DF_MESH_RELAY                   0
#define DF_MESH_DEVICE_NAME_PREFIX      "FIRE_NODE_"
#define DF_MESH_EVT_QUEUE_LEN           20              /* Chiều dài hàng đợi sự kiện   */
#define DF_MESH_TASK_STACK              4096
#define DF_MESH_TASK_PRIO               5
#define DF_MESH_LOG_DEPTH               32

/* Địa chỉ Group HOMI Fire Alarm */
#define DF_MESH_GROUP_FIRE_ALARM        0xC000

/* HOMI Network / AppKey Index */
#define DF_MESH_HOMI_NET_KEY_IDX        0x0000
#define DF_MESH_HOMI_APP_KEY_IDX_0      0x0000
#define DF_MESH_HOMI_APP_KEY_IDX_1      0x0001
#define DF_MESH_HOMI_APP_KEY_IDX_2      0x0002

/* Vendor Model Company ID (duplicate guard với app_mesh_vendor.h) */
#ifndef DF_MESH_VND_COMPANY_ID
#define DF_MESH_VND_COMPANY_ID          0x8888
#endif
#ifndef DF_MESH_VND_MODEL_ID_SERVER
#define DF_MESH_VND_MODEL_ID_SERVER     0x0000
#endif

/* ================== BẬT / TẮT MODULE ================== */
#define DF_ENABLE_BLE_MESH              1
#define DF_ENABLE_WIFI                  0
#define DF_ENABLE_BLUFI                 0
#define DF_ENABLE_MQTT                  0
#define DF_ENABLE_LITTLEFS              0

/* ================== MÃ LỖI ================== */
#define ERROR_CODE_OK                   50000
#define ERROR_CODE_FAIL                 50001
#define ERROR_CODE_UNDEFINED            50004
#define ERROR_CODE_INVALID_PARAM        50005
#define ERROR_CODE_TIMEOUT              50010
#define ERROR_CODE_NO_MEMORY            50011
#define ERROR_CODE_NOT_SUPPORT          50012
#define ERROR_CODE_BUSY                 50013
#define ERROR_CODE_NOT_FOUND            50014

#define DF_CHECK_NULL_PARAM(ptr)                                                      \
    do {                                                                              \
        if ((ptr) == NULL) {                                                          \
            ESP_LOGE("PARAM", "Loi con tro NULL tai %s:%d", __FUNCTION__, __LINE__);  \
            return ERROR_CODE_INVALID_PARAM;                                          \
        }                                                                             \
    } while (0)
