#include "app_common.h"

bool g_bDebug = true;

void app_common_DelayMs(uint32_t u32Ms)
{
    vTaskDelay(pdMS_TO_TICKS(u32Ms));
}

const char *app_common_GetVersion(void)
{
    return DF_FIRMWARE_VERSION;
}

void app_common_SetDebug(bool bEnable)
{
    g_bDebug = bEnable;
}