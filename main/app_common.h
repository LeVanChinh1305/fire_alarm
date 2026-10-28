#pragma once

#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ======================= LOG ==========================
extern bool g_bDebug;

#define LREP(tag, format, ...)          ESP_LOGI(tag, format, ##__VA_ARGS__)
#define LREP_WARNING(tag, format, ...)  ESP_LOGW(tag, format, ##__VA_ARGS__)
#define LREP_ERROR(tag, format, ...)    ESP_LOGE(tag, format, ##__VA_ARGS__)
#define LREP_DEBUG(tag, format, ...)    do { if (g_bDebug) ESP_LOGI(tag, format, ##__VA_ARGS__); } while (0)

// ======================= PROTOTYPE =====================
void app_common_DelayMs(uint32_t u32Ms);
const char *app_common_GetVersion(void);
void app_common_SetDebug(bool bEnable);

// ======================= QUẢN LÝ BỘ NHỚ ==================
#define FREE_SAFE(ptr) do {     \
    if ((ptr) != NULL) {        \
        heap_caps_free(ptr);    \
        (ptr) = NULL;           \
    }                           \
} while (0)

#define MALLOC_PSRAM(size)  heap_caps_calloc(1, (size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define MALLOC_DMA(size)    heap_caps_malloc((size), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)