#include "app_console.h"
#include "app_common.h"
#include "app_config.h"
#include "app_blemesh.h"

#include "esp_console.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PROMPT_STR                      "fire"
#define DF_CONSOLE_MAX_CMDLINE_LENGTH   128
#define DF_CONSOLE_TASK_STACK_SIZE      (4096)

/* ================== COMMAND HANDLERS ================== */

static int app_console_CmdVersion(int argc, char **argv)
{
    printf("\n=== Thông tin phần mềm ===\n");
    printf("Tên Firmware : %s\n", DF_FIRMWARE_NAME);
    printf("Phiên bản    : %s\n", app_common_GetVersion());
    printf("Target Chip  : %s\n", CONFIG_IDF_TARGET);
    printf("ESP-IDF Ver  : %s\n\n", esp_get_idf_version());
    return 0;
}

static int app_console_CmdRestart(int argc, char **argv){
    printf("Khởi động lại chip...\n");
    app_common_DelayMs(200);
    esp_restart();
    return 0;
}

static int app_console_CmdHeap(int argc, char **argv){
    size_t sram_free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t sram_min   = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    printf("\n=== Bộ nhớ hệ thống ===\n");
    printf("Internal SRAM Free : %u bytes (Min Free: %u bytes)\n",
           (unsigned int)sram_free, (unsigned int)sram_min);
    printf("External PSRAM Free: %u bytes\n\n", (unsigned int)psram_free);
    return 0;
}

static int app_console_CmdDebug(int argc, char **argv){
    if (argc < 2) {
        printf("Cú pháp: debug <0|1>\n");
        return 1;
    }
    int iVal = atoi(argv[1]);
    app_common_SetDebug(iVal != 0);
    printf("Chế độ debug đã chuyển thành: %s\n", g_bDebug ? "BAT (ON)" : "TAT (OFF)");
    return 0;
}

static int app_console_CmdNvsErase(int argc, char **argv){
    printf("\n>>> Tiến hành xóa sạch phân vùng nvs và khởi động lại ... <<<\n");
    nvs_flash_erase();
    app_common_DelayMs(300);
    esp_restart();
    return 0;
}

static int app_console_CmdMesh(int argc, char **argv){
    app_mesh_PrintStatus();
    return 0;
}

static int app_console_CmdMeshReset(int argc, char **argv){
    return (app_mesh_Reset() == ERROR_CODE_OK) ? 0 : 1;
}

static int app_console_CmdMeshJoin(int argc, char **argv){
    int32_t ret = app_mesh_GatewayJoin();
    if (ret == ERROR_CODE_OK)        { printf("GatewayJoin da bat dau (xem log BIND_COMP / SUB_COMP)\n"); return 0; }
    else if (ret == ERROR_CODE_BUSY) { printf("GatewayJoin dang chay...\n"); return 0; }
    else                             { printf("GatewayJoin that bai (chua provision?)\n"); return 1; }
}


/* ================== ĐĂNG KÝ LỆNH ================== */

static void app_console_RegisterCommands(void)
{
    const esp_console_cmd_t stCmdVer = {
        .command = "version",
        .help    = "Xem tên firmware và tên phiên bản ESP-IDF",
        .hint    = NULL,
        .func    = &app_console_CmdVersion,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdVer));

    const esp_console_cmd_t stCmdRe = {
        .command = "restart",
        .help    = "Khởi động lại ESP32",
        .hint    = NULL,
        .func    = &app_console_CmdRestart,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdRe));

    const esp_console_cmd_t stCmdHeap = {
        .command = "heap",
        .help    = "Kiểm tra dung lượng RAM/PSRAM còn trống",
        .hint    = NULL,
        .func    = &app_console_CmdHeap,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHeap));

    const esp_console_cmd_t stCmdDbg = {
        .command = "debug",
        .help    = "Bật/tắt log debug (debug <0|1>)",
        .hint    = NULL,
        .func    = &app_console_CmdDebug,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdDbg));

    const esp_console_cmd_t stCmdNvs = {
        .command = "nvserase",
        .help    = "Xóa sạch NVS Flash va restart",
        .hint    = NULL,
        .func    = &app_console_CmdNvsErase,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdNvs));

    const esp_console_cmd_t stCmdMesh = {
        .command = "mesh",
        .help    = "In trạng thái BLE Mesh",
        .hint    = NULL,
        .func    = &app_console_CmdMesh,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMesh));

    const esp_console_cmd_t stCmdMeshReset = {
        .command = "meshreset",
        .help    = "Reset về Unprovisioned",
        .hint    = NULL,
        .func    = &app_console_CmdMeshReset,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshReset));


    const esp_console_cmd_t stCmdMeshJoin = {
        .command = "meshjoin",
        .help    = "Tu dong them AppKey HOMI, bind va subscribe tat ca model",
        .hint    = NULL,
        .func    = &app_console_CmdMeshJoin,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshJoin));
}

/* ================== KHỞI TẠO CONSOLE ================== */

void app_console_Init(void)
{
    esp_console_repl_t *pRepl = NULL;
    esp_console_repl_config_t stReplConfig = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    stReplConfig.prompt = PROMPT_STR "> ";
    stReplConfig.max_cmdline_length = DF_CONSOLE_MAX_CMDLINE_LENGTH;
    stReplConfig.task_stack_size = DF_CONSOLE_TASK_STACK_SIZE;

    esp_console_register_help_command();
    app_console_RegisterCommands();

#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t stHwConfig = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&stHwConfig, &stReplConfig, &pRepl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t stHwConfig = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&stHwConfig, &stReplConfig, &pRepl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t stHwConfig = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&stHwConfig, &stReplConfig, &pRepl));
#else
    esp_console_dev_uart_config_t stHwConfig = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&stHwConfig, &stReplConfig, &pRepl));
#endif

    printf("\n");
    printf("==================================================\n");
    printf("   %s Console CLI v%s\n", DF_FIRMWARE_NAME, app_common_GetVersion());
    printf("   Go 'help' de xem tat ca cac lenh ho tro\n");
    printf("==================================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(pRepl));
}
