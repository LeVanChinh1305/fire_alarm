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

static int app_console_CmdMeshModels(int argc, char **argv){
    app_mesh_PrintModels();
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

/* ================== CLIENT COMMANDS ================== */

static int app_console_CmdOnOffGet(int argc, char **argv)
{
    if (argc < 2) {
        printf("Cú pháp: onoffget <unicast_addr_hex|dec>\n");
        printf("Ví dụ: onoffget 0x0102\n");
        return 1;
    }

    char *endptr = NULL;
    unsigned long addr = strtoul(argv[1], &endptr, 0);
    if (endptr == argv[1] || *endptr != '\0' || addr < 0x0001 || addr > 0xFEFF) {
        printf("Địa chỉ không hợp lệ: %s\n", argv[1]);
        return 1;
    }

    int32_t ret = app_mesh_SendOnOffGet((uint16_t)addr);
    if (ret == ERROR_CODE_OK) {
        printf("Đã gửi Generic OnOff Get tới 0x%04lX\n", addr);
        return 0;
    }
    printf("Gửi OnOff Get thất bại (lỗi %ld)\n", (long)ret);
    return 1;
}

static int app_console_CmdHealthGet(int argc, char **argv)
{
    if (argc < 2) {
        printf("Cú pháp: healthget <unicast_addr_hex> [company_id_hex]\n");
        printf("Ví dụ: healthget 0x0102 0x8888\n");
        return 1;
    }

    char *endptr = NULL;
    unsigned long addr = strtoul(argv[1], &endptr, 0);
    if (endptr == argv[1] || *endptr != '\0' || addr < 0x0001 || addr > 0x7FFF) {
        printf("Địa chỉ unicast không hợp lệ: %s\n", argv[1]);
        return 1;
    }

    uint16_t cid = DF_MESH_COMPANY_ID;
    if (argc >= 3) {
        cid = (uint16_t)strtoul(argv[2], NULL, 0);
    }

    int32_t ret = app_mesh_SendHealthFaultGet((uint16_t)addr, cid);
    if (ret == ERROR_CODE_OK) {
        printf("Đã gửi Health Fault Get tới 0x%04lX (cid=0x%04X)\n", addr, cid);
        return 0;
    }
    printf("Gửi Health Fault Get thất bại\n");
    return 1;
}

static int app_console_CmdHealthAttGet(int argc, char **argv)
{
    if (argc < 2) {
        printf("Cú pháp: healthattget <unicast_addr_hex>\n");
        return 1;
    }

    char *endptr = NULL;
    unsigned long addr = strtoul(argv[1], &endptr, 0);
    if (endptr == argv[1] || *endptr != '\0' || addr < 0x0001 || addr > 0x7FFF) {
        printf("Địa chỉ unicast không hợp lệ: %s\n", argv[1]);
        return 1;
    }

    int32_t ret = app_mesh_SendHealthAttentionGet((uint16_t)addr);
    if (ret == ERROR_CODE_OK) {
        printf("Đã gửi Health Attention Get tới 0x%04lX\n", addr);
        return 0;
    }
    printf("Gửi Attention Get thất bại\n");
    return 1;
}

static int app_console_CmdHealthPeriodGet(int argc, char **argv)
{
    if (argc < 2) {
        printf("Cú pháp: healthperiodget <unicast_addr_hex>\n");
        return 1;
    }

    char *endptr = NULL;
    unsigned long addr = strtoul(argv[1], &endptr, 0);
    if (endptr == argv[1] || *endptr != '\0' || addr < 0x0001 || addr > 0x7FFF) {
        printf("Địa chỉ unicast không hợp lệ: %s\n", argv[1]);
        return 1;
    }

    int32_t ret = app_mesh_SendHealthPeriodGet((uint16_t)addr);
    if (ret == ERROR_CODE_OK) {
        printf("Đã gửi Health Period Get tới 0x%04lX\n", addr);
        return 0;
    }
    printf("Gửi Period Get thất bại\n");
    return 1;
}

static bool app_console_ParseU32(const char *text, unsigned long max, unsigned long *value)
{
    char *endptr = NULL;
    unsigned long parsed;
    if (text == NULL || value == NULL) return false;
    parsed = strtoul(text, &endptr, 0);
    if (endptr == text || *endptr != '\0' || parsed > max) return false;
    *value = parsed;
    return true;
}

static int app_console_CmdHealthAttSet(int argc, char **argv)
{
    unsigned long addr, seconds;
    if (argc != 3 ||
        !app_console_ParseU32(argv[1], 0x7FFF, &addr) || addr == 0 ||
        !app_console_ParseU32(argv[2], 255, &seconds)) {
        printf("Cu phap: healthattset <unicast_addr> <seconds 0..255>\n");
        return 1;
    }
    return app_mesh_SendHealthAttentionSet((uint16_t)addr, (uint8_t)seconds) == ERROR_CODE_OK ? 0 : 1;
}

static int app_console_CmdHealthPeriodSet(int argc, char **argv)
{
    unsigned long addr, divisor;
    if (argc != 3 ||
        !app_console_ParseU32(argv[1], 0x7FFF, &addr) || addr == 0 ||
        !app_console_ParseU32(argv[2], 15, &divisor)) {
        printf("Cu phap: healthperiodset <unicast_addr> <divisor 0..15>\n");
        return 1;
    }
    return app_mesh_SendHealthPeriodSet((uint16_t)addr, (uint8_t)divisor) == ERROR_CODE_OK ? 0 : 1;
}

static int app_console_CmdHealthFaultClear(int argc, char **argv)
{
    unsigned long addr, cid;
    if (argc != 3 ||
        !app_console_ParseU32(argv[1], 0x7FFF, &addr) || addr == 0 ||
        !app_console_ParseU32(argv[2], 0xFFFF, &cid)) {
        printf("Cu phap: healthclear <unicast_addr> <company_id>\n");
        return 1;
    }
    return app_mesh_SendHealthFaultClear((uint16_t)addr, (uint16_t)cid) == ERROR_CODE_OK ? 0 : 1;
}

static int app_console_CmdHealthFaultTest(int argc, char **argv)
{
    unsigned long addr, cid, test_id;
    if (argc != 4 ||
        !app_console_ParseU32(argv[1], 0x7FFF, &addr) || addr == 0 ||
        !app_console_ParseU32(argv[2], 0xFFFF, &cid) ||
        !app_console_ParseU32(argv[3], 0xFF, &test_id)) {
        printf("Cu phap: healthtest <unicast_addr> <company_id> <test_id>\n");
        return 1;
    }
    return app_mesh_SendHealthFaultTest((uint16_t)addr, (uint16_t)cid, (uint8_t)test_id) == ERROR_CODE_OK ? 0 : 1;
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
        .help    = "In trạng thái BLE Mesh (Unicast, AppKey mask, Vendor RX count...)",
        .hint    = NULL,
        .func    = &app_console_CmdMesh,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMesh));

    const esp_console_cmd_t stCmdMeshModels = {
        .command = "meshmodels",
        .help    = "In chi tiet tung model: AppKey da bind, Group da subscribe",
        .hint    = NULL,
        .func    = &app_console_CmdMeshModels,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshModels));

    const esp_console_cmd_t stCmdMeshReset = {
        .command = "meshreset",
        .help    = "Reset về Unprovisioned",
        .hint    = NULL,
        .func    = &app_console_CmdMeshReset,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshReset));

    const esp_console_cmd_t stCmdMeshJoin = {
        .command = "meshjoin",
        .help    = "Them 3 AppKey HOMI, bind Health Client va subscribe Group 0xC000",
        .hint    = NULL,
        .func    = &app_console_CmdMeshJoin,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshJoin));

    /* Các lệnh Client Model */
    const esp_console_cmd_t stCmdOnOffGet = {
        .command = "onoffget",
        .help    = "Gửi Generic OnOff Get: onoffget <addr>",
        .hint    = NULL,
        .func    = &app_console_CmdOnOffGet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdOnOffGet));

    const esp_console_cmd_t stCmdHealthGet = {
        .command = "healthget",
        .help    = "Gửi Health Fault Get: healthget <addr> [cid]",
        .hint    = NULL,
        .func    = &app_console_CmdHealthGet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthGet));

    /* Alias 'health' trỏ về healthget để tương thích */
    const esp_console_cmd_t stCmdHealth = {
        .command = "health",
        .help    = "Alias của healthget: health <addr> [cid]",
        .hint    = NULL,
        .func    = &app_console_CmdHealthGet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealth));

    const esp_console_cmd_t stCmdHealthAttGet = {
        .command = "healthattget",
        .help    = "Gửi Health Attention Get: healthattget <addr>",
        .hint    = NULL,
        .func    = &app_console_CmdHealthAttGet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthAttGet));

    const esp_console_cmd_t stCmdHealthPeriodGet = {
        .command = "healthperiodget",
        .help    = "Gửi Health Period Get: healthperiodget <addr>",
        .hint    = NULL,
        .func    = &app_console_CmdHealthPeriodGet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthPeriodGet));

    const esp_console_cmd_t stCmdHealthAttSet = {
        .command = "healthattset",
        .help    = "Health Attention Set: healthattset <addr> <seconds 0..255>",
        .hint    = NULL,
        .func    = &app_console_CmdHealthAttSet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthAttSet));

    const esp_console_cmd_t stCmdHealthPeriodSet = {
        .command = "healthperiodset",
        .help    = "Health Period Set: healthperiodset <addr> <divisor 0..15>",
        .hint    = NULL,
        .func    = &app_console_CmdHealthPeriodSet,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthPeriodSet));

    const esp_console_cmd_t stCmdHealthClear = {
        .command = "healthclear",
        .help    = "Health Fault Clear: healthclear <addr> <company_id>",
        .hint    = NULL,
        .func    = &app_console_CmdHealthFaultClear,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthClear));

    const esp_console_cmd_t stCmdHealthTest = {
        .command = "healthtest",
        .help    = "Health Fault Test: healthtest <addr> <company_id> <test_id>",
        .hint    = NULL,
        .func    = &app_console_CmdHealthFaultTest,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealthTest));
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
