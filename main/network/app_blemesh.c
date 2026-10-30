/**
 * @file    app_blemesh.c
 * @brief   BLE Mesh Node – Fire Alarm S3
 *          Cấu trúc: 15 model Server (2 element) + Vendor Server 0x8888/0x0000
 *          Cải tiến so với phiên bản cũ:
 *            1. 3 AppKey HOMI thật (thay 0x00 cũ).
 *            2. GatewayJoin bất đồng bộ qua Queue + Task (tránh HCI timeout).
 *            3. Vendor callback rõ ràng hơn: log đầy đủ, đếm bản tin, parse cấu trúc.
 *            4. Subscribe Group 0xC000 cho OnOff Srv và Vendor Srv sau khi bind.
 *            5. Lưu/khôi phục cấu hình qua NVS.
 */

#include "app_blemesh.h"
#include "app_config.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_health_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "APP_MESH";

/* ============================================================
 *  DEFINES
 * ============================================================ */
#ifndef ROLE_NODE
#define ROLE_NODE 0
#endif

#define DF_LOCK_TIMEOUT_MS      500
#define NVS_NAMESPACE           "meshgw"
#define NVS_KEY_NETIDX          "netidx"
#define NVS_KEY_GROUP           "group"

/* ============================================================
 *  SỰ KIỆN NỘI BỘ (Queue + Task)
 * ============================================================ */
typedef enum {
    MESH_EVT_ENABLED = 0,       /* Mesh stack sẵn sàng (sau prov enable)      */
    MESH_EVT_PROV_COMPLETE,     /* Provision thành công                       */
    MESH_EVT_PROV_RESET,        /* Node bị reset                              */
    MESH_EVT_LOCAL_APPKEY_ADD,  /* Kết quả add AppKey cục bộ                 */
    MESH_EVT_LOCAL_APP_BIND,    /* Kết quả bind AppKey vào model              */
    MESH_EVT_LOCAL_SUB_ADD,     /* Kết quả subscribe Group                   */
    MESH_EVT_ONOFF_SET,         /* OnOff Server nhận SET                      */
    MESH_EVT_VENDOR_MSG,        /* Vendor Model nhận bản tin                  */
} en_mesh_evt_t;

typedef struct {
    en_mesh_evt_t eType;
    int64_t       s64TimeUs;
    uint16_t      u16Src;
    uint16_t      u16Dst;
    uint16_t      u16Param;
    uint16_t      u16ModelId;
    int           iErrCode;
    uint8_t       u8OnOff;
    uint8_t       u8Ttl;
    int8_t        s8Rssi;
} st_mesh_evt_t;

/* ============================================================
 *  BIẾN TRẠNG THÁI TOÀN CỤC
 * ============================================================ */
typedef struct {
    bool     bProvisioned;
    uint16_t u16Unicast;
    uint16_t u16NetIdx;
    uint16_t u16AppIdx;     /* AppIdx chính (HOMI_APP_KEY_IDX_0 = 0x0000) */
    bool     bAppBound;
    uint16_t u16Group;
    uint8_t  u8OnOffState;
    uint32_t u32RxVendorCount;  /* Đếm tổng số bản tin Vendor nhận được    */
} st_mesh_info_t;

static st_mesh_info_t    g_stInfo;
static bool              g_bMeshInitialized  = false;
static QueueHandle_t     g_hMeshEvtQ         = NULL;
static TaskHandle_t      g_hMeshTask         = NULL;
static SemaphoreHandle_t g_hMeshMutex        = NULL;
static uint32_t          g_u32DropEvt        = 0;

/* ---- Gateway Join state ---- */
static bool     g_bGatewayConfigStarted = false;
static bool     g_bGatewayJoined        = false;
static uint8_t  g_u8AppKeyReadyMask     = 0;       /* Bit i = AppKey[i] đã thêm */
static uint8_t  g_u8BindPendingCount    = 0;
static uint8_t  g_u8BindFailedCount     = 0;
static uint8_t  g_u8GroupPendingCount   = 0;
static bool     g_bGroupConfigFailed    = false;
static uint16_t g_u16ConfiguredGroup    = DF_MESH_GROUP_FIRE_ALARM;
static uint16_t g_u16ProvisionedNetIdx  = DF_MESH_HOMI_NET_KEY_IDX;

/* ============================================================
 *  3 APPKEY HOMI THẬT
 *  (Thay thế key 0x00 cũ – đây là keys tương thích với Gateway HOMI)
 * ============================================================ */
static const uint8_t HOMI_APP_KEY_0[16] = {
    0x16, 0xE7, 0xC9, 0x34, 0xEB, 0x3F, 0x5A, 0x8B,
    0xDB, 0x63, 0x2A, 0x09, 0x6A, 0xA8, 0x2C, 0xAC,
};
static const uint8_t HOMI_APP_KEY_1[16] = {
    0xF0, 0x7E, 0xE1, 0xB4, 0xEF, 0x3C, 0xFE, 0x00,
    0xD7, 0x08, 0x43, 0xF6, 0x3D, 0x0A, 0xBC, 0xAB,
};
static const uint8_t HOMI_APP_KEY_2[16] = {
    0xB3, 0xAA, 0x31, 0xD0, 0x7A, 0xFB, 0x8D, 0x21,
    0x54, 0x2D, 0x85, 0x35, 0x33, 0xE6, 0xC3, 0xDD,
};

/* ============================================================
 *  UUID THIẾT BỊ
 * ============================================================ */
static uint8_t g_au8DevUuid[16] = { 0x32, 0x10 };  /* MAC điền vào [2..7] khi init */

/* ============================================================
 *  CONFIG SERVER
 * ============================================================ */
static esp_ble_mesh_cfg_srv_t g_stCfgSrv = {
    .relay       = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon      = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy  = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy  = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .friend_state     = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .default_ttl      = 7,
    .net_transmit     = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/* ============================================================
 *  PUBLICATION BUFFERS (mỗi model một buffer riêng)
 * ============================================================ */
ESP_BLE_MESH_HEALTH_PUB_DEFINE(g_stHealthPub,       8,       ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stOnoffPub,         2 + 3,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLevelPub,         2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stDttPub,           2 + 1,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stPwrPub,           2 + 1,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stPwrSetupPub,      2 + 1,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLightnessPub,     2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLightnessSetupPub,2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlPub,           2 + 9,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlSetupPub,      2 + 9,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLevel2Pub,        2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlTempPub,       2 + 9,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stVndSrvPub,        3 + sizeof(st_vendor_custom_data_t), ROLE_NODE);

/* ============================================================
 *  HEALTH SERVER
 * ============================================================ */
static const uint8_t g_au8HealthTestIds[] = { 0x00 };
static esp_ble_mesh_health_srv_t g_stHealthSrv = {
    .health_test.id_count = ARRAY_SIZE(g_au8HealthTestIds),
    .health_test.test_ids = g_au8HealthTestIds,
    .health_test.company_id = DF_MESH_VND_COMPANY_ID,
};
static esp_ble_mesh_client_t g_stHealthCli;

/* ============================================================
 *  GENERIC SERVER STATES
 * ============================================================ */
#define DF_SRV_RSP_AUTO \
    { .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP, \
      .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP }

static esp_ble_mesh_gen_onpowerup_state_t g_stOnPowerUpState;

static esp_ble_mesh_gen_onoff_srv_t g_stOnoffSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};
static esp_ble_mesh_gen_level_srv_t g_stLevelSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};
static esp_ble_mesh_gen_level_srv_t g_stLevel2Srv = {    /* Element 1 */
    .rsp_ctrl = DF_SRV_RSP_AUTO
};
static esp_ble_mesh_gen_def_trans_time_srv_t g_stDttSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};
static esp_ble_mesh_gen_power_onoff_srv_t g_stPwrSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stOnPowerUpState
};
static esp_ble_mesh_gen_power_onoff_setup_srv_t g_stPwrSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stOnPowerUpState
};

/* ============================================================
 *  LIGHTING SERVER STATES
 * ============================================================ */
static esp_ble_mesh_light_lightness_state_t g_stLightnessState = {
    .lightness_range_min = 1,
    .lightness_range_max = 0xFFFF,
    .lightness_default   = 0xFFFF,
};
static esp_ble_mesh_light_ctl_state_t g_stCtlState = {
    .lightness_default     = 0xFFFF,
    .temperature           = 4000,
    .temperature_default   = 4000,
    .temperature_range_min = 800,
    .temperature_range_max = 20000,
};

static esp_ble_mesh_light_lightness_srv_t g_stLightnessSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stLightnessState
};
static esp_ble_mesh_light_lightness_setup_srv_t g_stLightnessSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stLightnessState
};
static esp_ble_mesh_light_ctl_srv_t g_stCtlSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stCtlState
};
static esp_ble_mesh_light_ctl_setup_srv_t g_stCtlSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stCtlState
};
static esp_ble_mesh_light_ctl_temp_srv_t g_stCtlTempSrv = { /* Element 1 */
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stCtlState
};

/* ============================================================
 *  VENDOR MODEL – đăng ký 64 opcode (0x00-0x3F)
 *  Chiến lược "lưới opcode" để hứng mọi bản tin Vendor của HOMI
 *  bất kể opcode byte nào Gateway gửi.
 * ============================================================ */
#define VND_OP(n) ESP_BLE_MESH_MODEL_OP(DF_VND_OPCODE(n), 0)
#define VND_OPS_8(b) \
    VND_OP((b)+0), VND_OP((b)+1), VND_OP((b)+2), VND_OP((b)+3), \
    VND_OP((b)+4), VND_OP((b)+5), VND_OP((b)+6), VND_OP((b)+7)

static esp_ble_mesh_model_op_t g_astVndServerOp[] = {
    VND_OPS_8(0x00), VND_OPS_8(0x08),
    VND_OPS_8(0x10), VND_OPS_8(0x18),
    VND_OPS_8(0x20), VND_OPS_8(0x28),
    VND_OPS_8(0x30), VND_OPS_8(0x38),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t g_astVndModels[] = {
    ESP_BLE_MESH_VENDOR_MODEL(
        DF_MESH_VND_COMPANY_ID,
        DF_MESH_VND_MODEL_ID_SERVER,
        g_astVndServerOp,
        &g_stVndSrvPub,
        NULL
    ),
};

/* ============================================================
 *  ELEMENT 0: 12 SIG Server + Health CLI  (13 models)  +  Vendor (1)
 *  ELEMENT 1: 2 SIG Server
 * ============================================================ */
static esp_ble_mesh_model_t g_astRootModels[] = {
    /* [0]  */ ESP_BLE_MESH_MODEL_CFG_SRV(&g_stCfgSrv),
    /* [1]  */ ESP_BLE_MESH_MODEL_HEALTH_SRV(&g_stHealthSrv, &g_stHealthPub),
    /* [2]  */ ESP_BLE_MESH_MODEL_HEALTH_CLI(&g_stHealthCli),
    /* [3]  */ ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&g_stOnoffPub, &g_stOnoffSrv),
    /* [4]  */ ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&g_stLevelPub, &g_stLevelSrv),
    /* [5]  */ ESP_BLE_MESH_MODEL_GEN_DEF_TRANS_TIME_SRV(&g_stDttPub, &g_stDttSrv),
    /* [6]  */ ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SRV(&g_stPwrPub, &g_stPwrSrv),
    /* [7]  */ ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SETUP_SRV(&g_stPwrSetupPub, &g_stPwrSetupSrv),
    /* [8]  */ ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SRV(&g_stLightnessPub, &g_stLightnessSrv),
    /* [9]  */ ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SETUP_SRV(&g_stLightnessSetupPub, &g_stLightnessSetupSrv),
    /* [10] */ ESP_BLE_MESH_MODEL_LIGHT_CTL_SRV(&g_stCtlPub, &g_stCtlSrv),
    /* [11] */ ESP_BLE_MESH_MODEL_LIGHT_CTL_SETUP_SRV(&g_stCtlSetupPub, &g_stCtlSetupSrv),
};

static esp_ble_mesh_model_t g_astSecModels[] = {
    /* [0]  */ ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&g_stLevel2Pub, &g_stLevel2Srv),
    /* [1]  */ ESP_BLE_MESH_MODEL_LIGHT_CTL_TEMP_SRV(&g_stCtlTempPub, &g_stCtlTempSrv),
};

static esp_ble_mesh_elem_t g_astElements[] = {
    ESP_BLE_MESH_ELEMENT(0, g_astRootModels, g_astVndModels),
    ESP_BLE_MESH_ELEMENT(0, g_astSecModels,  ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t g_stComp = {
    .cid           = DF_MESH_COMPANY_ID,
    .element_count = ARRAY_SIZE(g_astElements),
    .elements      = g_astElements,
};

static esp_ble_mesh_prov_t g_stProv = {
    .uuid = g_au8DevUuid,
};

/* ============================================================
 *  DANH SÁCH MODEL CẦN BIND / SUBSCRIBE
 *  (Config Server dùng DevKey – không bind AppKey, bỏ qua)
 * ============================================================ */
typedef struct {
    uint8_t  u8Elem;     /* 0 hoặc 1 */
    uint16_t u16Cid;     /* ESP_BLE_MESH_CID_NVAL = SIG model */
    uint16_t u16ModelId;
    bool     bSub;       /* true = cũng subscribe Group */
} df_join_item_t;

static const df_join_item_t g_astJoin[] = {
    /* Element 0 – SIG models */
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_SRV,                    true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_CLI,                    true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV,                 true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV,                 true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_DEF_TRANS_TIME_SRV,        true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_POWER_ONOFF_SRV,           true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_POWER_ONOFF_SETUP_SRV,     true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SRV,           true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV,     true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SRV,                 true  },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SETUP_SRV,           true  },
    /* Element 0 – Vendor model (quan trọng nhất!) */
    { 0, DF_MESH_VND_COMPANY_ID, DF_MESH_VND_MODEL_ID_SERVER,                        true  },
    /* Element 1 */
    { 1, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV,                 true  },
    { 1, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_TEMP_SRV,            true  },
};

/* ============================================================
 *  FORWARD DECLARATIONS
 * ============================================================ */
static void    app_mesh_PostEvt(const st_mesh_evt_t *pstEvt);
static void    app_mesh_Task(void *pvParameters);
static int32_t app_mesh_GatewayJoinNetwork(uint16_t u16NetIdx);
static void    app_mesh_StartGatewayModelConfig(void);
static void    app_mesh_UpdateGatewayReady(void);
static void    app_mesh_LoadConfig(void);
static void    app_mesh_SaveConfig(void);

/* ============================================================
 *  HELPER: Đẩy sự kiện vào Queue (gọi được từ callback ISR-context)
 * ============================================================ */
static void app_mesh_PostEvt(const st_mesh_evt_t *pstEvt)
{
    if (g_hMeshEvtQ == NULL) return;
    if (xQueueSend(g_hMeshEvtQ, pstEvt, 0) != pdTRUE) {
        g_u32DropEvt++;
        LREP_WARNING(TAG, "[EVT] Queue day, bo qua event type=%d (tong=%lu)",
                     (int)pstEvt->eType, (unsigned long)g_u32DropEvt);
    }
}

/* ============================================================
 *  HELPER: Kiểm tra model đã có AppKey chưa
 * ============================================================ */
static bool app_mesh_ModelHasAppKey(const esp_ble_mesh_model_t *pstModel, uint16_t u16AppIdx)
{
    for (size_t i = 0; i < ARRAY_SIZE(pstModel->keys); i++) {
        if (pstModel->keys[i] == u16AppIdx) return true;
    }
    return false;
}

/* ============================================================
 *  CALLBACK: Provisioning
 *  *** Chỉ PostEvt, KHÔNG làm gì nặng bên trong ***
 * ============================================================ */
static void app_mesh_ProvCb(esp_ble_mesh_prov_cb_event_t eEvent,
                            esp_ble_mesh_prov_cb_param_t *pstParam)
{
    st_mesh_evt_t stEvt = { .s64TimeUs = esp_timer_get_time() };

    switch (eEvent) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        LREP(TAG, "PROV_REGISTER_COMP err=%d", pstParam->prov_register_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        LREP(TAG, "[PROV] Node enable xong (err=%d)", pstParam->node_prov_enable_comp.err_code);
        stEvt.eType = MESH_EVT_ENABLED;
        app_mesh_PostEvt(&stEvt);
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        LREP(TAG, "[PROV] Link OPEN: %s",
             pstParam->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        LREP(TAG, "[PROV] Link CLOSE: %s",
             pstParam->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        LREP(TAG, "=== PROVISION HOAN TAT ===");
        LREP(TAG, "  NetIdx  : 0x%04X", pstParam->node_prov_complete.net_idx);
        LREP(TAG, "  Unicast : 0x%04X", pstParam->node_prov_complete.addr);
        stEvt.eType    = MESH_EVT_PROV_COMPLETE;
        stEvt.u16Src   = pstParam->node_prov_complete.net_idx;
        stEvt.u16Param = pstParam->node_prov_complete.addr;
        app_mesh_PostEvt(&stEvt);
        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        LREP_WARNING(TAG, "[PROV] Nhan su kien RESET provisioning");
        stEvt.eType = MESH_EVT_PROV_RESET;
        app_mesh_PostEvt(&stEvt);
        break;

    /* ---- Kết quả thêm AppKey cục bộ (do GatewayJoin gọi) ---- */
    case ESP_BLE_MESH_NODE_ADD_LOCAL_APP_KEY_COMP_EVT:
        LREP(TAG, "[JOIN] ADD_APPKEY: net=0x%04X app=0x%04X err=%d",
             pstParam->node_add_app_key_comp.net_idx,
             pstParam->node_add_app_key_comp.app_idx,
             pstParam->node_add_app_key_comp.err_code);
        stEvt.eType    = MESH_EVT_LOCAL_APPKEY_ADD;
        stEvt.u16Src   = pstParam->node_add_app_key_comp.net_idx;
        stEvt.u16Param = pstParam->node_add_app_key_comp.app_idx;
        stEvt.iErrCode = pstParam->node_add_app_key_comp.err_code;
        app_mesh_PostEvt(&stEvt);
        break;

    /* ---- Kết quả bind AppKey vào model cục bộ ---- */
    case ESP_BLE_MESH_NODE_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        LREP(TAG, "[JOIN] BIND_COMP: elem=0x%04X cid=0x%04X mod=0x%04X app=0x%04X err=%d",
             pstParam->node_bind_app_key_to_model_comp.element_addr,
             pstParam->node_bind_app_key_to_model_comp.company_id,
             pstParam->node_bind_app_key_to_model_comp.model_id,
             pstParam->node_bind_app_key_to_model_comp.app_idx,
             pstParam->node_bind_app_key_to_model_comp.err_code);
        stEvt.eType    = MESH_EVT_LOCAL_APP_BIND;
        stEvt.u16Src   = pstParam->node_bind_app_key_to_model_comp.element_addr;
        stEvt.u16Param = pstParam->node_bind_app_key_to_model_comp.app_idx;
        stEvt.u16ModelId = pstParam->node_bind_app_key_to_model_comp.model_id;
        stEvt.iErrCode = pstParam->node_bind_app_key_to_model_comp.err_code;
        app_mesh_PostEvt(&stEvt);
        break;

    /* ---- Kết quả subscribe Group ---- */
    case ESP_BLE_MESH_MODEL_SUBSCRIBE_GROUP_ADDR_COMP_EVT:
        LREP(TAG, "[JOIN] SUB_COMP: elem=0x%04X mod=0x%04X grp=0x%04X err=%d",
             pstParam->model_sub_group_addr_comp.element_addr,
             pstParam->model_sub_group_addr_comp.model_id,
             pstParam->model_sub_group_addr_comp.group_addr,
             pstParam->model_sub_group_addr_comp.err_code);
        stEvt.eType    = MESH_EVT_LOCAL_SUB_ADD;
        stEvt.u16Src   = pstParam->model_sub_group_addr_comp.element_addr;
        stEvt.u16ModelId = pstParam->model_sub_group_addr_comp.model_id;
        stEvt.u16Param = pstParam->model_sub_group_addr_comp.group_addr;
        stEvt.iErrCode = pstParam->model_sub_group_addr_comp.err_code;
        app_mesh_PostEvt(&stEvt);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Config Server
 *  Ghi nhận khi Provisioner thêm AppKey / bind model qua OTA.
 *  Sau khi Provisioner add AppKey → tự động gọi GatewayJoin.
 * ============================================================ */
static void app_mesh_ConfigServerCb(esp_ble_mesh_cfg_server_cb_event_t eEvent,
                                    esp_ble_mesh_cfg_server_cb_param_t *pstParam)
{
    if (eEvent != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) return;

    switch (pstParam->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        LREP(TAG, "[CFG SRV] APP_KEY_ADD: net=0x%04X app=0x%04X",
             pstParam->value.state_change.appkey_add.net_idx,
             pstParam->value.state_change.appkey_add.app_idx);
        /* Provisioner đã add AppKey → ta tự bind/subscribe thêm cho Vendor model */
        app_mesh_GatewayJoin();
        break;

    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        LREP(TAG, "[CFG SRV] MODEL_APP_BIND: elem=0x%04X app=0x%04X mod=0x%04X",
             pstParam->value.state_change.mod_app_bind.element_addr,
             pstParam->value.state_change.mod_app_bind.app_idx,
             pstParam->value.state_change.mod_app_bind.model_id);
        break;

    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        LREP(TAG, "[CFG SRV] MODEL_SUB_ADD: elem=0x%04X mod=0x%04X grp=0x%04X",
             pstParam->value.state_change.mod_sub_add.element_addr,
             pstParam->value.state_change.mod_sub_add.model_id,
             pstParam->value.state_change.mod_sub_add.sub_addr);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Generic Server
 * ============================================================ */
static void app_mesh_GenericServerCb(esp_ble_mesh_generic_server_cb_event_t eEvent,
                                     esp_ble_mesh_generic_server_cb_param_t *pstParam)
{
    if (pstParam == NULL || eEvent != ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT) return;

    st_mesh_evt_t stEvt = {
        .s64TimeUs = esp_timer_get_time(),
        .u16Src    = pstParam->ctx.addr,
        .u16Dst    = pstParam->ctx.recv_dst,
        .u8Ttl     = pstParam->ctx.recv_ttl,
    };

    switch (pstParam->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        uint8_t onoff = pstParam->value.state_change.onoff_set.onoff;
        uint16_t src  = pstParam->ctx.addr;
        uint16_t dst  = pstParam->ctx.recv_dst;
        int8_t  rssi  = pstParam->ctx.recv_rssi;
        uint8_t ttl   = pstParam->ctx.recv_ttl;

        bool bIsGroup = (dst >= 0xC000 && dst <= 0xFEFF);
        const char *pcType = bIsGroup ? "PUBLISH/GROUP" : "SET";

        LREP(TAG, "[ONOFF %s] src=0x%04X dst=0x%04X → %s | rssi=%d ttl=%u",
            pcType, src, dst, onoff ? "ON" : "OFF", rssi, ttl);

        /* Cập nhật state */
        g_stInfo.u8OnOffState = onoff;
        g_stOnoffSrv.state.onoff = onoff;

        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK:
        LREP(TAG, "[GEN] Level[elem %d] -> %d (src=0x%04X)",
             pstParam->model->element_idx,
             pstParam->value.state_change.level_set.level,
             pstParam->ctx.addr);
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_SET_UNACK:
        LREP(TAG, "[GEN] OnPowerUp -> %u", pstParam->value.state_change.onpowerup_set.onpowerup);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Lighting Server
 * ============================================================ */
static void app_mesh_LightingServerCb(esp_ble_mesh_lighting_server_cb_event_t eEvent,
                                      esp_ble_mesh_lighting_server_cb_param_t *pstParam)
{
    if (pstParam == NULL || eEvent != ESP_BLE_MESH_LIGHTING_SERVER_STATE_CHANGE_EVT) return;

    switch (pstParam->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET:
    case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK:
        LREP(TAG, "[LIGHT] Lightness -> %u", pstParam->value.state_change.lightness_set.lightness);
        break;
    case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET:
    case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET_UNACK:
        LREP(TAG, "[LIGHT] CTL -> lightness=%u temp=%u duv=%d",
             pstParam->value.state_change.ctl_set.lightness,
             pstParam->value.state_change.ctl_set.temperature,
             pstParam->value.state_change.ctl_set.delta_uv);
        break;
    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Vendor Model
 *  *** Quan trọng: KHÔNG block, KHÔNG delay, chỉ log + đếm ***
 *
 *  Lý do nhận vài bản tin rồi dừng ở bản cũ:
 *    - callback block quá lâu → HCI command timeout
 *    - Vendor model chưa được bind AppKey / subscribe Group đúng
 *  Giải pháp: log nhanh, cập nhật counter, rồi trả về ngay.
 * ============================================================ */
static void app_mesh_CustomModelCb(esp_ble_mesh_model_cb_event_t eEvent,
                                   esp_ble_mesh_model_cb_param_t *pstParam)
{
    if (eEvent != ESP_BLE_MESH_MODEL_OPERATION_EVT) return;
    if (pstParam == NULL ||
        pstParam->model_operation.model == NULL ||
        pstParam->model_operation.ctx   == NULL) return;

    /* Chỉ xử lý Vendor Model của mình */
    if (pstParam->model_operation.model->vnd.company_id != DF_MESH_VND_COMPANY_ID) return;

    uint32_t opcode  = pstParam->model_operation.opcode;
    uint16_t length  = pstParam->model_operation.length;
    uint8_t *data    = pstParam->model_operation.msg;
    uint16_t src     = pstParam->model_operation.ctx->addr;
    uint16_t dst     = pstParam->model_operation.ctx->recv_dst;
    uint8_t  ttl     = pstParam->model_operation.ctx->recv_ttl;
    int8_t   rssi    = pstParam->model_operation.ctx->recv_rssi;
    uint8_t  op_idx  = (uint8_t)((opcode >> 16) & 0x3F);

    /* ---- Cập nhật counter (mutex-free, chấp nhận race nhỏ trong context này) ---- */
    g_stInfo.u32RxVendorCount++;

    /* ---- Log nhanh một dòng (không printf nhiều dòng) ---- */
    LREP(TAG, "[VND RX #%lu] op=0x%02X src=0x%04X dst=0x%04X len=%u ttl=%u rssi=%d",
         (unsigned long)g_stInfo.u32RxVendorCount,
         op_idx, src, dst, length, ttl, rssi);

    /* ---- In hex payload (tối đa 16 byte để tránh chiếm quá lâu) ---- */
    if (length > 0 && data != NULL) {
        char acHex[64] = {0};
        uint16_t n = (length > 16) ? 16 : length;
        for (uint16_t i = 0; i < n; i++) {
            snprintf(acHex + i * 3, sizeof(acHex) - i * 3, "%02X ", data[i]);
        }
        LREP_DEBUG(TAG, "  Payload: %s%s", acHex, (length > 16) ? "..." : "");
    }

    /* ---- Parse theo OpIdx ---- */
    switch (op_idx) {
    case 0x01: /* SET – cấu trúc st_vendor_custom_data_t */
        if (length >= sizeof(st_vendor_custom_data_t) && data != NULL) {
            const st_vendor_custom_data_t *p = (const st_vendor_custom_data_t *)data;
            LREP(TAG, "  [SET] cmd=0x%02X pkt=%u temp=%.1f°C smoke=%u ppm alarm=%u tag=%.*s",
                 p->u8Cmd, p->u16PacketId,
                 (float)p->s16Temperature / 10.0f,
                 p->u16SmokeVal,
                 p->u8AlarmState,
                 (int)sizeof(p->acDeviceTag), p->acDeviceTag);
        }
        break;

    case 0x11: /* NODE_STATUS – nút bấm / đầu khói tự broadcast */
        if (length >= 2 && data != NULL) {
            // const char *dev_str = (data[0] == 0x01) ? "NUT BAM" :
            //                       (data[0] == 0x02) ? "DAU KHOI" : "UNKNOWN";
            // const char *st_str  = (data[1] == 0x01) ? "BINH THUONG" :
            //                       (data[1] == 0x02) ? "BAO DONG" :
            //                       (data[1] == 0x03) ? "KET THUC"  : "UNKNOWN";
            // if (length >= 8) {
            //     LREP(TAG, "  [NODE 0x11] %s | %s | MAC=%02X:%02X:%02X:%02X:%02X:%02X",
            //          dev_str, st_str,
            //          data[2], data[3], data[4], data[5], data[6], data[7]);
            // } else {
            //     LREP(TAG, "  [NODE 0x11] %s | %s", dev_str, st_str);
            // }
        }
        break;

    default:
        /* Bản tin vendor khác – đã log ở trên, không làm gì thêm */
        break;
    }
}


static void app_mesh_HealthClientCb(
    esp_ble_mesh_health_client_cb_event_t event,
    esp_ble_mesh_health_client_cb_param_t *param)
{
    if (param == NULL) {
        return;
    }

    uint16_t dst = 0;
    uint32_t opcode = 0;

    if (param->params != NULL) {
        dst = param->params->ctx.addr;
        opcode = param->params->opcode;
    }

    switch (event) {

    case ESP_BLE_MESH_HEALTH_CLIENT_GET_STATE_EVT:

        if (param->error_code != 0) {
            LREP_ERROR(TAG,
                "[HEALTH CLI] GET failed: dst=0x%04X err=%d",
                dst, param->error_code);
            break;
        }

        switch (opcode) {

        case ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_GET:
        {
            esp_ble_mesh_health_fault_status_cb_t *p =
                &param->status_cb.fault_status;

            LREP(TAG,
                "[HEALTH] FAULT STATUS: dst=0x%04X "
                "company=0x%04X test=0x%02X",
                dst, p->company_id, p->test_id);

            if (p->fault_array != NULL) {
                LREP(TAG, "[HEALTH] Fault count=%u",
                     p->fault_array->len);

                for (uint16_t i = 0;
                     i < p->fault_array->len; i++) {

                    LREP(TAG,
                        "[HEALTH] Fault[%u] = 0x%02X",
                        i, p->fault_array->data[i]);
                }
            }
            break;
        }

        case ESP_BLE_MESH_MODEL_OP_ATTENTION_GET:

            LREP(TAG,
                "[HEALTH] Attention=%u seconds",
                param->status_cb.attention_status.attention);
            break;

        case ESP_BLE_MESH_MODEL_OP_HEALTH_PERIOD_GET:

            LREP(TAG,
                "[HEALTH] Fast Period Divisor=%u",
                param->status_cb.period_status.fast_period_divisor);
            break;

        default:
            LREP(TAG,
                "[HEALTH] GET response opcode=0x%08lX",
                (unsigned long)opcode);
            break;
        }
        break;

    case ESP_BLE_MESH_HEALTH_CLIENT_SET_STATE_EVT:

        LREP(TAG,
            "[HEALTH CLI] SET event: dst=0x%04X "
            "opcode=0x%08lX err=%d",
            dst, (unsigned long)opcode, param->error_code);
        break;

    case ESP_BLE_MESH_HEALTH_CLIENT_PUBLISH_EVT:

        /* Chỉ xác nhận đã nhận Health Publish, không phân tích payload. */
        LREP(TAG,
            "[HEALTH CLI] *** RECEIVED HEALTH PUBLISH *** "
            "src=0x%04X opcode=0x%08lX",
            dst,
            (unsigned long)opcode);
        break;

    case ESP_BLE_MESH_HEALTH_CLIENT_TIMEOUT_EVT:

        LREP_ERROR(TAG,
            "[HEALTH CLI] TIMEOUT: dst=0x%04X opcode=0x%08lX",
            dst, (unsigned long)opcode);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  GATEWAY JOIN – Logic bất đồng bộ
 * ============================================================ */

/** Kiểm tra và cập nhật cờ g_bGatewayJoined */
static void app_mesh_UpdateGatewayReady(void)
{
    g_bGatewayJoined = g_bGatewayConfigStarted
                    && (g_u8AppKeyReadyMask == 0x07)
                    && (g_u8BindPendingCount == 0)
                    && (g_u8BindFailedCount  == 0)
                    && (g_u8GroupPendingCount == 0)
                    && !g_bGroupConfigFailed;

    if (g_bGatewayJoined) {
        LREP(TAG, "[GW] === GATEWAY JOIN HOAN TAT === Group=0x%04X", g_u16ConfiguredGroup);
        if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
            g_stInfo.bAppBound = true;
            g_stInfo.u16Group  = g_u16ConfiguredGroup;
            xSemaphoreGive(g_hMeshMutex);
        }
        app_mesh_SaveConfig();
    }

    /* Nếu cấu hình thất bại, reset cờ để cho phép thử lại */
    if (g_bGatewayConfigStarted &&
        (g_u8BindFailedCount != 0 || g_bGroupConfigFailed)) {
        LREP_ERROR(TAG, "[GW] GATEWAY JOIN THAT BAI: bind_fail=%u group_fail=%d",
                   g_u8BindFailedCount, (int)g_bGroupConfigFailed);
        g_bGatewayConfigStarted = false;
        g_bGatewayJoined        = false;
    }
}

/** Bind tất cả 3 AppKey vào từng model trong g_astJoin */
static void app_mesh_StartModelBindings(void)
{
    const uint16_t u16AppIdx[] = {
        DF_MESH_HOMI_APP_KEY_IDX_0,
        DF_MESH_HOMI_APP_KEY_IDX_1,
        DF_MESH_HOMI_APP_KEY_IDX_2,
    };

    g_u8BindPendingCount = 0;
    g_u8BindFailedCount  = 0;

    uint16_t u16PrimAddr = esp_ble_mesh_get_primary_element_address();

    for (size_t ki = 0; ki < ARRAY_SIZE(u16AppIdx); ki++) {
        for (size_t mi = 0; mi < ARRAY_SIZE(g_astJoin); mi++) {
            uint16_t u16ElemAddr = u16PrimAddr + g_astJoin[mi].u8Elem;

            /* Tìm con trỏ model để kiểm tra AppKey đã có chưa */
            esp_ble_mesh_model_t *pstModel = NULL;
            if (g_astJoin[mi].u8Elem == 0) {
                if (g_astJoin[mi].u16Cid == DF_MESH_VND_COMPANY_ID) {
                    pstModel = &g_astVndModels[0];
                } else {
                    for (size_t m = 0; m < ARRAY_SIZE(g_astRootModels); m++) {
                        if (g_astRootModels[m].model_id == g_astJoin[mi].u16ModelId) {
                            pstModel = &g_astRootModels[m]; break;
                        }
                    }
                }
            } else {
                for (size_t m = 0; m < ARRAY_SIZE(g_astSecModels); m++) {
                    if (g_astSecModels[m].model_id == g_astJoin[mi].u16ModelId) {
                        pstModel = &g_astSecModels[m]; break;
                    }
                }
            }

            if (pstModel && app_mesh_ModelHasAppKey(pstModel, u16AppIdx[ki])) {
                continue; /* Đã bind – bỏ qua */
            }

            esp_err_t err = esp_ble_mesh_node_bind_app_key_to_local_model(
                u16ElemAddr,
                g_astJoin[mi].u16Cid,
                g_astJoin[mi].u16ModelId,
                u16AppIdx[ki]
            );
            if (err == ESP_OK) {
                g_u8BindPendingCount++;
            } else {
                g_u8BindFailedCount++;
                LREP_ERROR(TAG, "[GW] Queue bind AppKey=0x%03X model=0x%04X fail: %s",
                           u16AppIdx[ki], g_astJoin[mi].u16ModelId, esp_err_to_name(err));
            }
        }
    }

    LREP(TAG, "[GW] Queued %u bind ops", g_u8BindPendingCount);
}

/** Subscribe Group cho Health Client, OnOff Server và Vendor Server. */
static void app_mesh_StartGroupSubscribe(uint16_t u16GroupAddr)
{
    uint16_t u16Prim = esp_ble_mesh_get_primary_element_address();
    g_u8GroupPendingCount = 0;
    g_bGroupConfigFailed  = false;

    typedef struct {
        esp_ble_mesh_model_t *pst;
        uint16_t cid;
        uint16_t mid;
    } sub_target_t;

    sub_target_t aTargets[] = {
        /* Health Client nhận Health Current Status Publish */
        { &g_astRootModels[2], ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_CLI },
        /* Các model đang subscribe từ trước */
        { &g_astRootModels[3], ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV },
        { &g_astVndModels[0],  DF_MESH_VND_COMPANY_ID, DF_MESH_VND_MODEL_ID_SERVER },
    };

    for (size_t i = 0; i < ARRAY_SIZE(aTargets); i++) {
        if (esp_ble_mesh_is_model_subscribed_to_group(
                aTargets[i].pst, u16GroupAddr) != NULL) {
            LREP(TAG, "[GW] Model 0x%04X already subscribed to 0x%04X",
                 aTargets[i].mid, u16GroupAddr);
            continue;
        }

        esp_err_t err = esp_ble_mesh_model_subscribe_group_addr(
            u16Prim, aTargets[i].cid, aTargets[i].mid, u16GroupAddr);
        if (err == ESP_OK) {
            g_u8GroupPendingCount++;
            LREP(TAG, "[GW] Subscribe queued: model=0x%04X group=0x%04X",
                 aTargets[i].mid, u16GroupAddr);
        } else {
            g_bGroupConfigFailed = true;
            LREP_ERROR(TAG, "[GW] Subscribe grp 0x%04X model=0x%04X fail: %s",
                       u16GroupAddr, aTargets[i].mid, esp_err_to_name(err));
        }
    }

    LREP(TAG, "[GW] Queued %u subscribe ops", g_u8GroupPendingCount);
}

/** Được gọi khi tất cả 3 AppKey đã thêm thành công */
static void app_mesh_StartGatewayModelConfig(void)
{
    if (!g_bGatewayConfigStarted || g_u8AppKeyReadyMask != 0x07) return;
    LREP(TAG, "[GW] Tất cả AppKey sẵn sàng – bắt đầu Bind + Subscribe");
    app_mesh_StartModelBindings();
    app_mesh_StartGroupSubscribe(g_u16ConfiguredGroup);
    app_mesh_UpdateGatewayReady();
}

/* ============================================================
 *  NVS: Lưu / Tải cấu hình
 * ============================================================ */
static void app_mesh_LoadConfig(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;

    uint16_t v = 0;
    if (nvs_get_u16(h, NVS_KEY_NETIDX, &v) == ESP_OK && v <= 0x0FFF) {
        g_u16ProvisionedNetIdx = v;
    }
    if (nvs_get_u16(h, NVS_KEY_GROUP, &v) == ESP_OK && v >= 0xC000 && v <= 0xFEFF) {
        g_u16ConfiguredGroup = v;
    }
    nvs_close(h);
    LREP(TAG, "[NVS] Load: NetIdx=0x%04X Group=0x%04X",
         g_u16ProvisionedNetIdx, g_u16ConfiguredGroup);
}

static void app_mesh_SaveConfig(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u16(h, NVS_KEY_NETIDX, g_u16ProvisionedNetIdx);
    nvs_set_u16(h, NVS_KEY_GROUP,  g_u16ConfiguredGroup);
    nvs_commit(h);
    nvs_close(h);
    LREP(TAG, "[NVS] Save: NetIdx=0x%04X Group=0x%04X",
         g_u16ProvisionedNetIdx, g_u16ConfiguredGroup);
}

/* ============================================================
 *  TASK XỬ LÝ SỰ KIỆN
 *  Tất cả logic nặng (bind, subscribe, delay, v.v.) chạy ở đây
 *  để callback (chạy trên BT task) trả về nhanh nhất có thể.
 * ============================================================ */
static void app_mesh_Task(void *pvParameters)
{
    st_mesh_evt_t stEvt;

    while (1) {
        if (xQueueReceive(g_hMeshEvtQ, &stEvt, portMAX_DELAY) != pdTRUE) continue;

        switch (stEvt.eType) {

        /* Mesh stack đã sẵn sàng – nếu đã provision thì tự join */
        case MESH_EVT_ENABLED: {
            bool bProv = esp_ble_mesh_node_is_provisioned();
            uint16_t u16Addr = bProv ? esp_ble_mesh_get_primary_element_address() : 0;
            if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
                g_stInfo.bProvisioned = bProv;
                g_stInfo.u16Unicast   = u16Addr;
                g_stInfo.u16NetIdx    = g_u16ProvisionedNetIdx;
                xSemaphoreGive(g_hMeshMutex);
            }
            LREP(TAG, "[MESH] Stack sẵn sàng. Provisioned=%s Unicast=0x%04X",
                 bProv ? "YES" : "NO", u16Addr);
            if (bProv) {
                vTaskDelay(pdMS_TO_TICKS(500)); /* Chờ link provisioning đóng hẳn */
                app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx);
            }
            break;
        }

        /* Vừa được provision */
        case MESH_EVT_PROV_COMPLETE: {
            uint16_t u16NetIdx = stEvt.u16Src;
            uint16_t u16Addr   = stEvt.u16Param;
            g_u16ProvisionedNetIdx = u16NetIdx;
            if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
                g_stInfo.bProvisioned = true;
                g_stInfo.u16NetIdx    = u16NetIdx;
                g_stInfo.u16Unicast   = u16Addr;
                xSemaphoreGive(g_hMeshMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); /* Chờ provisioning link đóng hẳn */
            app_mesh_GatewayJoinNetwork(u16NetIdx);
            break;
        }

        /* Reset node */
        case MESH_EVT_PROV_RESET:
            LREP_WARNING(TAG, "[MESH] Reset provisioning – restarting...");
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
            break;

        /* AppKey đã thêm xong */
        case MESH_EVT_LOCAL_APPKEY_ADD: {
            int iKeyPos = -1;
            if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_0) iKeyPos = 0;
            if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_1) iKeyPos = 1;
            if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_2) iKeyPos = 2;

            if (stEvt.iErrCode != ESP_OK || iKeyPos < 0) {
                LREP_ERROR(TAG, "[GW] AppKey add fail: app=0x%04X err=%d",
                           stEvt.u16Param, stEvt.iErrCode);
                g_bGatewayConfigStarted = false;
                break;
            }
            g_u8AppKeyReadyMask |= (uint8_t)(1U << iKeyPos);
            LREP(TAG, "[GW] AppKey[%d] sẵn sàng. Mask=0x%02X", iKeyPos, g_u8AppKeyReadyMask);
            app_mesh_StartGatewayModelConfig();
            break;
        }

        /* Bind AppKey vào model xong */
        case MESH_EVT_LOCAL_APP_BIND:
            /* Chỉ đếm các bind của element gốc */
            if (stEvt.u16Src == esp_ble_mesh_get_primary_element_address() ||
                stEvt.u16Src == esp_ble_mesh_get_primary_element_address() + 1) {
                if (g_u8BindPendingCount > 0) g_u8BindPendingCount--;
                if (stEvt.iErrCode != ESP_OK) g_u8BindFailedCount++;
            }
            app_mesh_UpdateGatewayReady();
            break;

        /* Subscribe Group xong */
        case MESH_EVT_LOCAL_SUB_ADD:
            if (g_u8GroupPendingCount > 0) g_u8GroupPendingCount--;
            if (stEvt.iErrCode != ESP_OK) g_bGroupConfigFailed = true;
            app_mesh_UpdateGatewayReady();
            break;

        /* OnOff Server nhận lệnh SET */
        case MESH_EVT_ONOFF_SET:
            if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
                g_stInfo.u8OnOffState = stEvt.u8OnOff;
                xSemaphoreGive(g_hMeshMutex);
            }
            g_stOnoffSrv.state.onoff = stEvt.u8OnOff;
            break;

        default:
            break;
        }
    }
}

/* ============================================================
 *  app_mesh_GatewayJoinNetwork  – thêm 3 AppKey, trigger bind
 * ============================================================ */
static int32_t app_mesh_GatewayJoinNetwork(uint16_t u16NetIdx)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) {
        LREP_WARNING(TAG, "[GW] Chua provision, bo qua GatewayJoin");
        return ERROR_CODE_FAIL;
    }
    if (esp_ble_mesh_node_get_local_net_key(u16NetIdx) == NULL) {
        LREP_ERROR(TAG, "[GW] NetKey 0x%04X khong ton tai", u16NetIdx);
        return ERROR_CODE_NOT_FOUND;
    }
    if (g_bGatewayConfigStarted) {
        return g_bGatewayJoined ? ERROR_CODE_OK : ERROR_CODE_BUSY;
    }

    LREP(TAG, "[GW] Bat dau GatewayJoin (NetIdx=0x%04X)", u16NetIdx);

    const uint8_t  *pKeys[]     = { HOMI_APP_KEY_0, HOMI_APP_KEY_1, HOMI_APP_KEY_2 };
    const uint16_t  u16AppIdx[] = { DF_MESH_HOMI_APP_KEY_IDX_0,
                                    DF_MESH_HOMI_APP_KEY_IDX_1,
                                    DF_MESH_HOMI_APP_KEY_IDX_2 };

    g_bGatewayConfigStarted = true;
    g_bGatewayJoined        = false;
    g_u8AppKeyReadyMask     = 0;
    g_u8BindPendingCount    = 0;
    g_u8BindFailedCount     = 0;
    g_u8GroupPendingCount   = 0;
    g_bGroupConfigFailed    = false;

    for (size_t i = 0; i < 3; i++) {
        /* Kiểm tra AppKey đã tồn tại chưa */
        const uint8_t *pExisting = esp_ble_mesh_node_get_local_app_key(u16AppIdx[i]);
        if (pExisting != NULL) {
            if (memcmp(pExisting, pKeys[i], 16) == 0) {
                /* Đúng key – đánh dấu sẵn sàng ngay */
                g_u8AppKeyReadyMask |= (uint8_t)(1U << i);
                LREP(TAG, "[GW] AppKey[%zu] (0x%04X) da ton tai, skip add", i, u16AppIdx[i]);
                continue;
            } else {
                LREP_ERROR(TAG, "[GW] AppKey[%zu] (0x%04X) ton tai nhung KHAC – kich ban loi!",
                           i, u16AppIdx[i]);
                g_bGatewayConfigStarted = false;
                return ERROR_CODE_INVALID_PARAM;
            }
        }

        esp_err_t err = esp_ble_mesh_node_add_local_app_key(pKeys[i], u16NetIdx, u16AppIdx[i]);
        if (err != ESP_OK) {
            LREP_ERROR(TAG, "[GW] Add AppKey[%zu] fail: %s", i, esp_err_to_name(err));
            g_bGatewayConfigStarted = false;
            return ERROR_CODE_FAIL;
        }
        /* Kết quả sẽ về qua MESH_EVT_LOCAL_APPKEY_ADD */
    }

    /* Nếu tất cả key đã tồn tại → bắt đầu bind ngay */
    if (g_u8AppKeyReadyMask == 0x07) {
        app_mesh_StartGatewayModelConfig();
    }

    g_u16ProvisionedNetIdx = u16NetIdx;
    return ERROR_CODE_OK;
}

/* ============================================================
 *  PUBLIC API
 * ============================================================ */

int32_t app_mesh_GatewayJoin(void)
{
    return app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx);
}

bool app_mesh_GetGatewayJoined(void)
{
    return g_bGatewayJoined;
}

bool app_mesh_IsProvisioned(void)
{
    return g_stInfo.bProvisioned ||
           (g_bMeshInitialized && esp_ble_mesh_node_is_provisioned());
}

uint8_t app_mesh_GetOnOffState(void)
{
    uint8_t v = 0;
    if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
        v = g_stInfo.u8OnOffState;
        xSemaphoreGive(g_hMeshMutex);
    }
    return v;
}

/* ============================================================
 *  app_mesh_Init
 * ============================================================ */
int32_t app_mesh_Init(void)
{
    if (g_bMeshInitialized) return ERROR_CODE_OK;

    /* ---- Tạo Mutex và Queue ---- */
    g_hMeshMutex = xSemaphoreCreateMutex();
    if (!g_hMeshMutex) return ERROR_CODE_NO_MEMORY;

    g_hMeshEvtQ = xQueueCreate(DF_MESH_EVT_QUEUE_LEN, sizeof(st_mesh_evt_t));
    if (!g_hMeshEvtQ) return ERROR_CODE_NO_MEMORY;

    memset(&g_stInfo, 0, sizeof(g_stInfo));
    g_stInfo.u16NetIdx = DF_MESH_HOMI_NET_KEY_IDX;
    g_stInfo.u16AppIdx = DF_MESH_HOMI_APP_KEY_IDX_0;

    /* ---- Load cấu hình từ NVS ---- */
    app_mesh_LoadConfig();

    /* ---- Tạo Task xử lý sự kiện ---- */
    if (xTaskCreate(app_mesh_Task, "mesh_task",
                    DF_MESH_TASK_STACK, NULL,
                    DF_MESH_TASK_PRIO, &g_hMeshTask) != pdPASS) {
        return ERROR_CODE_FAIL;
    }

    /* ---- UUID = {0x32, 0x10, MAC[0..5], 0..} ---- */
    uint8_t au8Mac[6] = {0};
    esp_read_mac(au8Mac, ESP_MAC_BT);
    memcpy(&g_au8DevUuid[2], au8Mac, sizeof(au8Mac));
    LREP(TAG, "Device UUID MAC: %02X:%02X:%02X:%02X:%02X:%02X",
         au8Mac[0], au8Mac[1], au8Mac[2], au8Mac[3], au8Mac[4], au8Mac[5]);

    /* ---- Khởi tạo Bluetooth ---- */
#if !CONFIG_IDF_TARGET_ESP32
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err;
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) { LREP_ERROR(TAG, "bt_ctrl_init: %s", esp_err_to_name(err)); return ERROR_CODE_FAIL; }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) { LREP_ERROR(TAG, "bt_ctrl_enable: %s", esp_err_to_name(err)); return ERROR_CODE_FAIL; }
    err = esp_bluedroid_init();
    if (err != ESP_OK) { LREP_ERROR(TAG, "bluedroid_init: %s", esp_err_to_name(err)); return ERROR_CODE_FAIL; }
    err = esp_bluedroid_enable();
    if (err != ESP_OK) { LREP_ERROR(TAG, "bluedroid_enable: %s", esp_err_to_name(err)); return ERROR_CODE_FAIL; }

    /* ---- Đăng ký Callbacks (trước esp_ble_mesh_init) ---- */
    esp_ble_mesh_register_prov_callback(app_mesh_ProvCb);
    esp_ble_mesh_register_config_server_callback(app_mesh_ConfigServerCb);
    esp_ble_mesh_register_generic_server_callback(app_mesh_GenericServerCb);
    esp_ble_mesh_register_lighting_server_callback(app_mesh_LightingServerCb);
    esp_ble_mesh_register_custom_model_callback(app_mesh_CustomModelCb);
    err = esp_ble_mesh_register_health_client_callback(app_mesh_HealthClientCb);

    if (err != ESP_OK) {
        LREP_ERROR(TAG,
            "Register Health Client callback failed: %s",
            esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    /* ---- Đặt tên thiết bị ---- */
    char acName[32];
    snprintf(acName, sizeof(acName), "%s%02X%02X",
             DF_MESH_DEVICE_NAME_PREFIX, au8Mac[4], au8Mac[5]);

    /* ---- Khởi tạo BLE Mesh stack ---- */
    err = esp_ble_mesh_init(&g_stProv, &g_stComp);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "esp_ble_mesh_init fail: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    esp_ble_mesh_set_unprovisioned_device_name(acName);

    /* ---- Bật Provisioning (PB-ADV + PB-GATT) ---- */
    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "prov_enable fail: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    g_bMeshInitialized = true;

    bool bProv = esp_ble_mesh_node_is_provisioned();
    LREP(TAG, "BLE Mesh khoi tao OK. Name=%s Prov=%s", acName, bProv ? "YES" : "NO");

    return ERROR_CODE_OK;
}

/* ============================================================
 *  app_mesh_Reset
 * ============================================================ */
int32_t app_mesh_Reset(void)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;
    esp_err_t err = esp_ble_mesh_node_local_reset();
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "node_local_reset fail: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
        memset(&g_stInfo, 0, sizeof(g_stInfo));
        xSemaphoreGive(g_hMeshMutex);
    }
    g_bGatewayConfigStarted = false;
    g_bGatewayJoined        = false;
    g_u8AppKeyReadyMask     = 0;
    LREP(TAG, "Da reset mesh ve Unprovisioned");
    return ERROR_CODE_OK;
}

/* ============================================================
 *  PRINT STATUS / MODELS (Console commands)
 * ============================================================ */
void app_mesh_PrintStatus(void)
{
    st_mesh_info_t snap;
    if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
        snap = g_stInfo;
        xSemaphoreGive(g_hMeshMutex);
    } else {
        snap = g_stInfo;
    }

    bool bProv    = app_mesh_IsProvisioned();
    uint16_t addr = bProv ? esp_ble_mesh_get_primary_element_address() : 0;

    printf("\n========================================\n");
    printf("       THONG TIN BLE MESH FIRE NODE     \n");
    printf("========================================\n");
    printf("Trang thai     : %s\n", bProv ? "DA PROVISION" : "CHUA PROVISION");
    printf("Unicast Addr   : 0x%04X\n", addr);
    printf("NetIdx / AppIdx: 0x%04X / 0x%04X\n", snap.u16NetIdx, snap.u16AppIdx);
    printf("GatewayJoined  : %s\n", g_bGatewayJoined ? "YES" : "NO");
    printf("AppKey Mask    : 0x%02X (7=OK)\n", g_u8AppKeyReadyMask);
    printf("Group Sub'd    : 0x%04X\n", snap.u16Group);
    printf("OnOff State    : %s\n", snap.u8OnOffState ? "ON" : "OFF");
    printf("Vendor RX cnt  : %lu\n", (unsigned long)snap.u32RxVendorCount);
    printf("Evt Drop count : %lu\n", (unsigned long)g_u32DropEvt);
    printf("========================================\n\n");
}

static void app_mesh_PrintModelInfo(const char *pcElemLabel,
                                    const esp_ble_mesh_model_t *pModels,
                                    size_t nModels, bool bVendor)
{
    for (size_t i = 0; i < nModels; i++) {
        uint16_t u16Id  = bVendor ? pModels[i].vnd.model_id : pModels[i].model_id;
        uint16_t u16Cid = bVendor ? pModels[i].vnd.company_id : 0xFFFF;

        printf("  [%s] %s id=0x%04X cid=0x%04X\n",
               pcElemLabel,
               bVendor ? "VND" : "SIG",
               u16Id, u16Cid);

        /* In AppKey đã bind */
        printf("    Keys  :");
        bool bAnyKey = false;
        for (size_t k = 0; k < ARRAY_SIZE(pModels[i].keys); k++) {
            if (pModels[i].keys[k] != 0xFFFF) {
                printf(" 0x%04X", pModels[i].keys[k]);
                bAnyKey = true;
            }
        }
        if (!bAnyKey) printf(" (chua bind)");
        printf("\n");

        /* In Group đã subscribe */
        printf("    Groups:");
        bool bAnyGrp = false;
        for (size_t g = 0; g < ARRAY_SIZE(pModels[i].groups); g++) {
            if (pModels[i].groups[g] != 0 && pModels[i].groups[g] != 0xFFFF) {
                printf(" 0x%04X", pModels[i].groups[g]);
                bAnyGrp = true;
            }
        }
        if (!bAnyGrp) printf(" (chua subscribe)");
        printf("\n");
    }
}

void app_mesh_PrintModels(void)
{
    printf("\n=== CHI TIET MODELS (keys=0xFFFF nghia la chua bind) ===\n");
    app_mesh_PrintModelInfo("elem0-SIG", g_astRootModels, ARRAY_SIZE(g_astRootModels), false);
    app_mesh_PrintModelInfo("elem0-VND", g_astVndModels,  ARRAY_SIZE(g_astVndModels),  true);
    app_mesh_PrintModelInfo("elem1-SIG", g_astSecModels,  ARRAY_SIZE(g_astSecModels),  false);
    printf("========================================================\n\n");
}


int32_t app_mesh_HealthFaultGet(uint16_t dst)
{
    if (!app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "Mesh node is not provisioned");
        return ERROR_CODE_FAIL;
    }

    if (dst < 0x0001 || dst > 0x7FFF) {
        LREP_ERROR(TAG, "Invalid unicast address: 0x%04X", dst);
        return ERROR_CODE_FAIL;
    }

    esp_ble_mesh_client_common_param_t common = {0};

    esp_ble_mesh_health_client_get_state_t get = {0};

    /* Opcode Health Fault Get */
    common.opcode =
        ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_GET;

    /* Health Client Model ở Element 0, index 2 */
    common.model = &g_astRootModels[2];

    /* Thông tin mạng */
    common.ctx.net_idx = g_u16ProvisionedNetIdx;

    /* AppKey đã được GatewayJoin thêm */
    common.ctx.app_idx = DF_MESH_HOMI_APP_KEY_IDX_0;

    /* Địa chỉ thiết bị Health Server cần kiểm tra */
    common.ctx.addr = dst;

    common.ctx.send_ttl = 7;
    common.ctx.send_rel = false;

    /* Timeout 5 giây */
    common.msg_timeout = 5000;

    /* Company ID của Health Server */
    get.fault_get.company_id = DF_MESH_COMPANY_ID;

    esp_err_t err = esp_ble_mesh_health_client_get_state(
                        &common, &get);

    if (err != ESP_OK) {
        LREP_ERROR(TAG,
            "Health Fault Get send failed: %s",
            esp_err_to_name(err));

        return ERROR_CODE_FAIL;
    }

    LREP(TAG,
        "[HEALTH] Fault Get sent to 0x%04X",
        dst);

    return ERROR_CODE_OK;
}