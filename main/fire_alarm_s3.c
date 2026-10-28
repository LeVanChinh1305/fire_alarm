#include "app_common.h"
#include "app_config.h"
#include "app_console.h"
#include "app_blemesh.h"


#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

static const char *TAG = "APP_MAIN";

static void app_main_InitNvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        LREP_WARNING(TAG, "NVS cần được erase do đổi phiên bản...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    LREP(TAG, "========================================");
    LREP(TAG, "  %s  v%s", DF_FIRMWARE_NAME, app_common_GetVersion());
    LREP(TAG, "  Target : %s | HW: %s", CONFIG_IDF_TARGET, DF_HARDWARE_VERSION);
    LREP(TAG, "========================================");

    // Khởi tạo NVS 
    app_main_InitNvs();

    // khởi tạo BLE mesh 
    if (app_mesh_Init() != ERROR_CODE_OK) {
        LREP_ERROR(TAG, "Khởi dộng BLE Mesh THẤT BẠI!");
    } else {
        LREP(TAG, "Khoi dong BLE Mesh THANH CONG.");
    }

    // Khởi tạo console 
    app_console_Init();

    // vòng lặp chính 
    uint32_t u32Count100ms = 0;
    while (1) {
        u32Count100ms++;

        if (u32Count100ms % 600 == 0) {   // mỗi 60 giây
            size_t sram_free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

            LREP_DEBUG("HEARTBEAT", "Free SRAM: %u KB | PSRAM: %u KB | Mesh: %s",
                       (unsigned)(sram_free / 1024),
                       (unsigned)(psram_free / 1024),
                       app_mesh_IsProvisioned() ? "Provisioned" : "Unprovisioned");
        }

        app_common_DelayMs(100);
    }
}