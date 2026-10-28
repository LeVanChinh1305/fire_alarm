set(MAIN_SRCS
    "fire_alarm_s3.c"
    "app_common.c"
    "console/app_console.c"
    "network/app_blemesh.c"
)

idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS
        "."
        "console"
        "network"
    REQUIRES
        freertos
        console
        esp_timer
        driver
        nvs_flash
        bt
)