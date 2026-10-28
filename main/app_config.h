#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"

/* ================== THÔNG TIN FIRMWARE ================== */
#define DF_FIRMWARE_VERSION             "1.0.0"
#define DF_FIRMWARE_NAME                "FIRE_ALARM_S3"
#define DF_DEVICE_TYPE                  4001
#define DF_HARDWARE_VERSION             "HW_V1.0"
#define DF_TARGET_CHIP                  CONFIG_IDF_TARGET

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

