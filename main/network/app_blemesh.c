/**
 * @file    app_blemesh.c
 * @brief   BLE Mesh Gateway Node – Fire Alarm S3
 *          Cấu trúc: Client Models (OnOff, Level, DTT, PowerOnOff, Lightness, CTL, Health)
 *                   + Server Models (Config, Health Srv, Setup Servers, Elem1 Temp)
 *                   + Vendor Server 0x8888/0x0000
 *          Giai doan hien tai:
 *            1. Health Client nhan Health Current Status Publish tu Group 0xC000.
 *            2. Log metadata va FaultArray, chua xu ly nghiep vu.
 *            3. Ho tro Health Fault/Attention/Period GET va cac lenh SET tu xa.
 *            4. GatewayJoin bat dong bo; chi bind Health Client trong giai doan nay.
 *            5. Cac model SIG/Vendor khac van giu trong Composition, chua bind/subscribe.
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
#define DF_MESH_HEALTH_GROUP_ADDR 0xC000

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
    uint8_t  u8OnOffState;      /* Cache trạng thái OnOff nhận từ OnOff Status */
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
static uint16_t g_u16ConfiguredGroup    = DF_MESH_HEALTH_GROUP_ADDR;
static uint16_t g_u16ProvisionedNetIdx  = DF_MESH_HOMI_NET_KEY_IDX;

/* ============================================================
 *  3 APPKEY HOMI THẬT
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
 *  PUBLICATION BUFFERS
 * ============================================================ */
ESP_BLE_MESH_HEALTH_PUB_DEFINE(g_stHealthPub,        8,      ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stOnoffCliPub,       2 + 3,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLevelCliPub,       2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stDttCliPub,         2 + 1,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stPwrCliPub,         2 + 1,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stPwrSetupPub,       2 + 1,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLightnessCliPub,   2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLightnessSetupPub, 2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlCliPub,         2 + 9,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlSetupPub,       2 + 9,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLevel2Pub,         2 + 5,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlTempPub,        2 + 9,  ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stVndSrvPub,         3 + sizeof(st_vendor_custom_data_t), ROLE_NODE);

/* ============================================================
 *  HEALTH SERVER & CLIENT
 * ============================================================ */
static const uint8_t g_au8HealthTestIds[] = { 0x00 };
static esp_ble_mesh_health_srv_t g_stHealthSrv = {
    .health_test.id_count = ARRAY_SIZE(g_au8HealthTestIds),
    .health_test.test_ids = g_au8HealthTestIds,
    .health_test.company_id = DF_MESH_VND_COMPANY_ID,
};
static esp_ble_mesh_client_t g_stHealthCli;

/* ============================================================
 *  GENERIC & LIGHTING CLIENT CONTEXTS
 * ============================================================ */
static esp_ble_mesh_client_t g_stOnoffCli;
static esp_ble_mesh_client_t g_stLevelCli;
static esp_ble_mesh_client_t g_stDttCli;
static esp_ble_mesh_client_t g_stPwrOnoffCli;
static esp_ble_mesh_client_t g_stLightnessCli;
static esp_ble_mesh_client_t g_stCtlCli;

/* ============================================================
 *  REMAINING SERVER STATES (Chỉ các Server không có SIG Client)
 * ============================================================ */
#define DF_SRV_RSP_AUTO \
    { .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP, \
      .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP }

static esp_ble_mesh_gen_onpowerup_state_t g_stOnPowerUpState;

static esp_ble_mesh_gen_power_onoff_setup_srv_t g_stPwrSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stOnPowerUpState
};

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

static esp_ble_mesh_light_lightness_setup_srv_t g_stLightnessSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stLightnessState
};
static esp_ble_mesh_light_ctl_setup_srv_t g_stCtlSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stCtlState
};

/* Element 1 */
static esp_ble_mesh_gen_level_srv_t g_stLevel2Srv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};
static esp_ble_mesh_light_ctl_temp_srv_t g_stCtlTempSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state    = &g_stCtlState
};

/* ============================================================
 *  VENDOR MODEL – đăng ký 64 opcode (0x00-0x3F)
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
 *  COMPOSITION DATA
 *  Element 0: Config Srv, Health Srv+Cli, OnOff Cli, Level Cli,
 *             DTT Cli, Pwr Cli, PwrSetup Srv, Lightness Cli,
 *             LightnessSetup Srv, CTL Cli, CTL Setup Srv + Vendor Srv
 *  Element 1: Level Srv, CTL Temp Srv
 * ============================================================ */
static esp_ble_mesh_model_t g_astRootModels[] = {
    /* [0]  */ ESP_BLE_MESH_MODEL_CFG_SRV(&g_stCfgSrv),
    /* [1]  */ ESP_BLE_MESH_MODEL_HEALTH_SRV(&g_stHealthSrv, &g_stHealthPub),
    /* [2]  */ ESP_BLE_MESH_MODEL_HEALTH_CLI(&g_stHealthCli),
    /* [3]  */ ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&g_stOnoffCliPub, &g_stOnoffCli),
    /* [4]  */ ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(&g_stLevelCliPub, &g_stLevelCli),
    /* [5]  */ ESP_BLE_MESH_MODEL_GEN_DEF_TRANS_TIME_CLI(&g_stDttCliPub, &g_stDttCli),
    /* [6]  */ ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_CLI(&g_stPwrCliPub, &g_stPwrOnoffCli),
    /* [7]  */ ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SETUP_SRV(&g_stPwrSetupPub, &g_stPwrSetupSrv),
    /* [8]  */ ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(&g_stLightnessCliPub, &g_stLightnessCli),
    /* [9]  */ ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SETUP_SRV(&g_stLightnessSetupPub, &g_stLightnessSetupSrv),
    /* [10] */ ESP_BLE_MESH_MODEL_LIGHT_CTL_CLI(&g_stCtlCliPub, &g_stCtlCli),
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
 * ============================================================ */
typedef struct {
    uint8_t  u8Elem;     /* 0 hoặc 1 */
    uint16_t u16Cid;     /* ESP_BLE_MESH_CID_NVAL = SIG model */
    uint16_t u16ModelId;
} df_join_item_t;

static const df_join_item_t g_astJoin[] = {
    /* Giai doan hien tai: chi bind Health Client de nhan Health Publish. */
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_CLI },
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
 *  HELPER: Đẩy sự kiện vào Queue
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
 *  CALLBACK: Generic Client
 * ============================================================ */
static void app_mesh_GenericClientCb(esp_ble_mesh_generic_client_cb_event_t eEvent,
                                     esp_ble_mesh_generic_client_cb_param_t *pstParam)
{
    if (!pstParam) return;

    uint16_t src = pstParam->params ? pstParam->params->ctx.addr : 0;
    uint16_t dst = pstParam->params ? pstParam->params->ctx.recv_dst : 0;
    uint32_t op  = pstParam->params ? pstParam->params->opcode : 0;
    int err      = pstParam->error_code;

    switch (eEvent) {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
        LREP(TAG, "[GEN CLI %s] op=0x%04lX src=0x%04X dst=0x%04X err=%d",
             (eEvent == ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT) ? "PUB" : "RSP",
             (unsigned long)op, src, dst, err);

        if (err != ESP_OK) {
            LREP_ERROR(TAG, "[GEN CLI] Command failed with err=%d", err);
            break;
        }

        if (op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS) {
            uint8_t onoff = pstParam->status_cb.onoff_status.present_onoff;
            LREP(TAG, "  [ONOFF STATUS] src=0x%04X onoff=%s", src, onoff ? "ON" : "OFF");
            if (g_hMeshMutex && xSemaphoreTake(g_hMeshMutex, pdMS_TO_TICKS(DF_LOCK_TIMEOUT_MS))) {
                g_stInfo.u8OnOffState = onoff;
                xSemaphoreGive(g_hMeshMutex);
            }
        } else if (op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS) {
            int16_t level = pstParam->status_cb.level_status.present_level;
            LREP(TAG, "  [LEVEL STATUS] src=0x%04X level=%d", src, level);
        } else if (op == ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_STATUS) {
            uint8_t powerup = pstParam->status_cb.onpowerup_status.onpowerup;
            LREP(TAG, "  [POWERUP STATUS] src=0x%04X powerup=%u", src, powerup);
        } else if (op == ESP_BLE_MESH_MODEL_OP_GEN_DEF_TRANS_TIME_STATUS) {
            uint8_t trans_time = pstParam->status_cb.def_trans_time_status.trans_time;
            LREP(TAG, "  [DTT STATUS] src=0x%04X trans_time=%u", src, trans_time);
        }
        break;

    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        LREP_WARNING(TAG, "[GEN CLI] TIMEOUT op=0x%04lX dst=0x%04X",
                     (unsigned long)op, dst);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Lighting Client
 * ============================================================ */
static void app_mesh_LightingClientCb(esp_ble_mesh_light_client_cb_event_t eEvent,
                                      esp_ble_mesh_light_client_cb_param_t *pstParam)
{
    if (!pstParam) return;

    uint16_t src = pstParam->params ? pstParam->params->ctx.addr : 0;
    uint16_t dst = pstParam->params ? pstParam->params->ctx.recv_dst : 0;
    uint32_t op  = pstParam->params ? pstParam->params->opcode : 0;
    int err      = pstParam->error_code;

    switch (eEvent) {
    case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT:
        LREP(TAG, "[LIGHT CLI %s] op=0x%04lX src=0x%04X dst=0x%04X err=%d",
             (eEvent == ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT) ? "PUB" : "RSP",
             (unsigned long)op, src, dst, err);

        if (err != ESP_OK) {
            LREP_ERROR(TAG, "[LIGHT CLI] Command failed with err=%d", err);
            break;
        }

        if (op == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_STATUS) {
            uint16_t lightness = pstParam->status_cb.lightness_status.present_lightness;
            LREP(TAG, "  [LIGHTNESS STATUS] src=0x%04X lightness=%u", src, lightness);
        } else if (op == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_STATUS) {
            uint16_t lightness = pstParam->status_cb.ctl_status.present_ctl_lightness;
            uint16_t temp      = pstParam->status_cb.ctl_status.present_ctl_temperature;
            LREP(TAG, "  [CTL STATUS] src=0x%04X lightness=%u temp=%u", src, lightness, temp);
        }
        break;

    case ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT:
        LREP_WARNING(TAG, "[LIGHT CLI] TIMEOUT op=0x%04lX dst=0x%04X",
                     (unsigned long)op, dst);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Health Client
 * ============================================================ */
static void app_mesh_HealthClientCb(esp_ble_mesh_health_client_cb_event_t event,
                                    esp_ble_mesh_health_client_cb_param_t *param)
{
    if (param == NULL) return;

    uint16_t peer_addr = 0;
    uint16_t recv_dst = 0;
    uint32_t opcode = 0;

    if (param->params != NULL) {
        peer_addr = param->params->ctx.addr;
        recv_dst = param->params->ctx.recv_dst;
        opcode = param->params->opcode;
    }

    switch (event) {
    case ESP_BLE_MESH_HEALTH_CLIENT_GET_STATE_EVT:
        if (param->error_code != ESP_OK) {
            LREP_ERROR(TAG, "[HEALTH CLI] GET failed: peer=0x%04X err=%d", peer_addr, param->error_code);
            break;
        }

        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_GET: {
            esp_ble_mesh_health_fault_status_cb_t *p = &param->status_cb.fault_status;
            LREP(TAG, "[HEALTH FAULT STATUS] peer=0x%04X company=0x%04X test=0x%02X",
                 peer_addr, p->company_id, p->test_id);
            if (p->fault_array != NULL) {
                LREP(TAG, "[HEALTH FAULT STATUS] count=%u", p->fault_array->len);
                for (uint16_t i = 0; i < p->fault_array->len; i++) {
                    LREP(TAG, "[HEALTH FAULT STATUS] fault[%u]=0x%02X", i, p->fault_array->data[i]);
                }
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_ATTENTION_GET:
            LREP(TAG, "[HEALTH ATTENTION STATUS] peer=0x%04X attention=%u seconds",
                 peer_addr, param->status_cb.attention_status.attention);
            break;
        case ESP_BLE_MESH_MODEL_OP_HEALTH_PERIOD_GET:
            LREP(TAG, "[HEALTH PERIOD STATUS] peer=0x%04X divisor=%u",
                 peer_addr, param->status_cb.period_status.fast_period_divisor);
            break;
        default:
            LREP(TAG, "[HEALTH GET STATUS] peer=0x%04X opcode=0x%08lX",
                 peer_addr, (unsigned long)opcode);
            break;
        }
        break;

    case ESP_BLE_MESH_HEALTH_CLIENT_SET_STATE_EVT:
        LREP(TAG, "[HEALTH SET STATUS] peer=0x%04X opcode=0x%08lX err=%d",
             peer_addr, (unsigned long)opcode, param->error_code);
        break;

    case ESP_BLE_MESH_HEALTH_CLIENT_PUBLISH_EVT: {
        /* Health Publish mang Health Current Status: doc current_status. */
        esp_ble_mesh_health_current_status_cb_t *p = &param->status_cb.current_status;
        LREP(TAG, "========== HEALTH PUBLISH RX ==========");
        LREP(TAG, "SRC=0x%04X DST=0x%04X OPCODE=0x%08lX ERR=%d",
             peer_addr, recv_dst, (unsigned long)opcode, param->error_code);
        LREP(TAG, "COMPANY=0x%04X TEST_ID=0x%02X",
             p->company_id, p->test_id);
        if (p->fault_array != NULL) {
            LREP(TAG, "FAULT_COUNT=%u", p->fault_array->len);
            for (uint16_t i = 0; i < p->fault_array->len; i++) {
                LREP(TAG, "FAULT[%u]=0x%02X", i, p->fault_array->data[i]);
            }
        } else {
            LREP(TAG, "FAULT_ARRAY=NULL");
        }
        LREP(TAG, "========================================");
        break;
    }

    case ESP_BLE_MESH_HEALTH_CLIENT_TIMEOUT_EVT:
        LREP_ERROR(TAG, "[HEALTH CLI] TIMEOUT: peer=0x%04X opcode=0x%08lX",
                   peer_addr, (unsigned long)opcode);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  CALLBACK: Vendor Model
 * ============================================================ */
static void app_mesh_CustomModelCb(esp_ble_mesh_model_cb_event_t eEvent,
                                   esp_ble_mesh_model_cb_param_t *pstParam)
{
    if (eEvent != ESP_BLE_MESH_MODEL_OPERATION_EVT) return;
    if (pstParam == NULL ||
        pstParam->model_operation.model == NULL ||
        pstParam->model_operation.ctx   == NULL) return;

    if (pstParam->model_operation.model->vnd.company_id != DF_MESH_VND_COMPANY_ID) return;

    uint32_t opcode  = pstParam->model_operation.opcode;
    uint16_t length  = pstParam->model_operation.length;
    uint8_t *data    = pstParam->model_operation.msg;
    uint16_t src     = pstParam->model_operation.ctx->addr;
    uint16_t dst     = pstParam->model_operation.ctx->recv_dst;
    uint8_t  ttl     = pstParam->model_operation.ctx->recv_ttl;
    int8_t   rssi    = pstParam->model_operation.ctx->recv_rssi;
    uint8_t  op_idx  = (uint8_t)((opcode >> 16) & 0x3F);

    g_stInfo.u32RxVendorCount++;

    LREP(TAG, "[VND RX #%lu] op=0x%02X src=0x%04X dst=0x%04X len=%u ttl=%u rssi=%d",
         (unsigned long)g_stInfo.u32RxVendorCount,
         op_idx, src, dst, length, ttl, rssi);

    if (length > 0 && data != NULL) {
        char acHex[64] = {0};
        uint16_t n = (length > 16) ? 16 : length;
        for (uint16_t i = 0; i < n; i++) {
            snprintf(acHex + i * 3, sizeof(acHex) - i * 3, "%02X ", data[i]);
        }
        LREP_DEBUG(TAG, "  Payload: %s%s", acHex, (length > 16) ? "..." : "");
    }

    switch (op_idx) {
    case 0x01:
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

    case 0x11:
        if (length >= 2 && data != NULL) {
            /* Node status message */
        }
        break;

    default:
        break;
    }
}

/* ============================================================
 *  GATEWAY JOIN – Logic bất đồng bộ
 * ============================================================ */
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

    if (g_bGatewayConfigStarted &&
        (g_u8BindFailedCount != 0 || g_bGroupConfigFailed)) {
        LREP_ERROR(TAG, "[GW] GATEWAY JOIN THAT BAI: bind_fail=%u group_fail=%d",
                   g_u8BindFailedCount, (int)g_bGroupConfigFailed);
        g_bGatewayConfigStarted = false;
        g_bGatewayJoined        = false;
    }
}

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
                continue;
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
        /* Giai doan 1: chi nhan Health Current Status Publish. */
        { &g_astRootModels[2], ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_CLI },
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
    /* Giai doan Health RX co dinh Group 0xC000 theo cau hinh gateway. */
    g_u16ConfiguredGroup = DF_MESH_HEALTH_GROUP_ADDR;
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
 * ============================================================ */
static void app_mesh_Task(void *pvParameters)
{
    st_mesh_evt_t stEvt;

    while (1) {
        if (xQueueReceive(g_hMeshEvtQ, &stEvt, portMAX_DELAY) != pdTRUE) continue;

        switch (stEvt.eType) {
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
                vTaskDelay(pdMS_TO_TICKS(500));
                app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx);
            }
            break;
        }

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
            vTaskDelay(pdMS_TO_TICKS(1000));
            app_mesh_GatewayJoinNetwork(u16NetIdx);
            break;
        }

        case MESH_EVT_PROV_RESET:
            LREP_WARNING(TAG, "[MESH] Reset provisioning – restarting...");
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
            break;

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

        case MESH_EVT_LOCAL_APP_BIND:
            if (stEvt.u16Src == esp_ble_mesh_get_primary_element_address() ||
                stEvt.u16Src == esp_ble_mesh_get_primary_element_address() + 1) {
                if (g_u8BindPendingCount > 0) g_u8BindPendingCount--;
                if (stEvt.iErrCode != ESP_OK) g_u8BindFailedCount++;
            }
            app_mesh_UpdateGatewayReady();
            break;

        case MESH_EVT_LOCAL_SUB_ADD:
            if (g_u8GroupPendingCount > 0) g_u8GroupPendingCount--;
            if (stEvt.iErrCode != ESP_OK) g_bGroupConfigFailed = true;
            app_mesh_UpdateGatewayReady();
            break;

        default:
            break;
        }
    }
}

/* ============================================================
 *  app_mesh_GatewayJoinNetwork
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
        const uint8_t *pExisting = esp_ble_mesh_node_get_local_app_key(u16AppIdx[i]);
        if (pExisting != NULL) {
            if (memcmp(pExisting, pKeys[i], 16) == 0) {
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
    }

    if (g_u8AppKeyReadyMask == 0x07) {
        app_mesh_StartGatewayModelConfig();
    }

    g_u16ProvisionedNetIdx = u16NetIdx;
    return ERROR_CODE_OK;
}

/* ============================================================
 *  PUBLIC API: General
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
 *  PUBLIC API: Client Commands (Chủ động gửi Get tới Remote Node)
 * ============================================================ */

int32_t app_mesh_SendOnOffGet(uint16_t u16DstAddr)
{
    if (!app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "[CLI] Node chua duoc provision");
        return ERROR_CODE_FAIL;
    }
    if (u16DstAddr < 0x0001 || u16DstAddr > 0xFEFF) {
        LREP_ERROR(TAG, "[CLI] Dia chi khong hop le: 0x%04X", u16DstAddr);
        return ERROR_CODE_INVALID_PARAM;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET,
        .model       = &g_astRootModels[3],   /* Generic OnOff Client ở RootModels[3] */
        .ctx.net_idx = g_u16ProvisionedNetIdx,
        .ctx.app_idx = DF_MESH_HOMI_APP_KEY_IDX_0,
        .ctx.addr    = u16DstAddr,
        .ctx.send_ttl = 7,
        .ctx.send_rel = false,
        .msg_timeout = 5000,
    };
    esp_ble_mesh_generic_client_get_state_t get = {0};

    esp_err_t err = esp_ble_mesh_generic_client_get_state(&common, &get);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "[CLI] OnOff Get toi 0x%04X that bai: %s", u16DstAddr, esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    LREP(TAG, "[CLI] Da gui OnOff Get toi 0x%04X", u16DstAddr);
    return ERROR_CODE_OK;
}

int32_t app_mesh_SendHealthFaultGet(uint16_t u16DstAddr, uint16_t u16CompanyId)
{
    if (!app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "[CLI] Node chua duoc provision");
        return ERROR_CODE_FAIL;
    }
    if (u16DstAddr < 0x0001 || u16DstAddr > 0x7FFF) {
        LREP_ERROR(TAG, "[CLI] Unicast khong hop le: 0x%04X", u16DstAddr);
        return ERROR_CODE_INVALID_PARAM;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_GET,
        .model       = &g_astRootModels[2],   /* Health Client ở RootModels[2] */
        .ctx.net_idx = g_u16ProvisionedNetIdx,
        .ctx.app_idx = DF_MESH_HOMI_APP_KEY_IDX_0,
        .ctx.addr    = u16DstAddr,
        .ctx.send_ttl = 7,
        .ctx.send_rel = false,
        .msg_timeout = 5000,
    };
    esp_ble_mesh_health_client_get_state_t get = {
        .fault_get.company_id = (u16CompanyId != 0) ? u16CompanyId : DF_MESH_COMPANY_ID,
    };

    esp_err_t err = esp_ble_mesh_health_client_get_state(&common, &get);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "[CLI] Health Fault Get toi 0x%04X that bai: %s", u16DstAddr, esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    LREP(TAG, "[CLI] Da gui Health Fault Get toi 0x%04X (cid=0x%04X)", u16DstAddr, get.fault_get.company_id);
    return ERROR_CODE_OK;
}

int32_t app_mesh_HealthFaultGet(uint16_t dst)
{
    return app_mesh_SendHealthFaultGet(dst, DF_MESH_COMPANY_ID);
}

int32_t app_mesh_SendHealthAttentionGet(uint16_t u16DstAddr)
{
    if (!app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "[CLI] Node chua duoc provision");
        return ERROR_CODE_FAIL;
    }
    if (u16DstAddr < 0x0001 || u16DstAddr > 0x7FFF) {
        LREP_ERROR(TAG, "[CLI] Unicast khong hop le: 0x%04X", u16DstAddr);
        return ERROR_CODE_INVALID_PARAM;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_ATTENTION_GET,
        .model       = &g_astRootModels[2],   /* Health Client */
        .ctx.net_idx = g_u16ProvisionedNetIdx,
        .ctx.app_idx = DF_MESH_HOMI_APP_KEY_IDX_0,
        .ctx.addr    = u16DstAddr,
        .ctx.send_ttl = 7,
        .ctx.send_rel = false,
        .msg_timeout = 5000,
    };
    esp_ble_mesh_health_client_get_state_t get = {0};

    esp_err_t err = esp_ble_mesh_health_client_get_state(&common, &get);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "[CLI] Attention Get toi 0x%04X that bai: %s", u16DstAddr, esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    LREP(TAG, "[CLI] Da gui Attention Get toi 0x%04X", u16DstAddr);
    return ERROR_CODE_OK;
}

int32_t app_mesh_SendHealthPeriodGet(uint16_t u16DstAddr)
{
    if (!app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "[CLI] Node chua duoc provision");
        return ERROR_CODE_FAIL;
    }
    if (u16DstAddr < 0x0001 || u16DstAddr > 0x7FFF) {
        LREP_ERROR(TAG, "[CLI] Unicast khong hop le: 0x%04X", u16DstAddr);
        return ERROR_CODE_INVALID_PARAM;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_HEALTH_PERIOD_GET,
        .model       = &g_astRootModels[2],
        .ctx.net_idx = g_u16ProvisionedNetIdx,
        .ctx.app_idx = DF_MESH_HOMI_APP_KEY_IDX_0,
        .ctx.addr    = u16DstAddr,
        .ctx.send_ttl = 7,
        .ctx.send_rel = false,
        .msg_timeout = 5000,
    };
    esp_ble_mesh_health_client_get_state_t get = {0};

    esp_err_t err = esp_ble_mesh_health_client_get_state(&common, &get);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "[CLI] Health Period Get toi 0x%04X that bai: %s", u16DstAddr, esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    LREP(TAG, "[CLI] Da gui Health Period Get toi 0x%04X", u16DstAddr);
    return ERROR_CODE_OK;
}

/* ============================================================
 *  PUBLIC API: Health Client SET commands
 * ============================================================ */
static int32_t app_mesh_HealthSetCommon(uint16_t dst, uint32_t opcode,
                                        esp_ble_mesh_health_client_set_state_t *set)
{
    if (!app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "[HEALTH] Node chua duoc provision");
        return ERROR_CODE_FAIL;
    }
    if (dst < 0x0001 || dst > 0x7FFF || set == NULL) {
        LREP_ERROR(TAG, "[HEALTH] Tham so khong hop le: dst=0x%04X", dst);
        return ERROR_CODE_INVALID_PARAM;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = opcode,
        .model        = &g_astRootModels[2],
        .ctx.net_idx  = g_u16ProvisionedNetIdx,
        .ctx.app_idx  = DF_MESH_HOMI_APP_KEY_IDX_0,
        .ctx.addr     = dst,
        .ctx.send_ttl = 7,
        .ctx.send_rel = false,
        .msg_timeout  = 5000,
    };

    esp_err_t err = esp_ble_mesh_health_client_set_state(&common, set);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "[HEALTH] SET opcode=0x%08lX dst=0x%04X fail: %s",
                   (unsigned long)opcode, dst, esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    LREP(TAG, "[HEALTH] SET opcode=0x%08lX sent to 0x%04X",
         (unsigned long)opcode, dst);
    return ERROR_CODE_OK;
}

int32_t app_mesh_SendHealthAttentionSet(uint16_t dst, uint8_t seconds)
{
    esp_ble_mesh_health_client_set_state_t set = {0};
    set.attention_set.attention = seconds;
    return app_mesh_HealthSetCommon(dst, ESP_BLE_MESH_MODEL_OP_ATTENTION_SET, &set);
}

int32_t app_mesh_SendHealthPeriodSet(uint16_t dst, uint8_t divisor)
{
    if (divisor > 0x0F) return ERROR_CODE_INVALID_PARAM;
    esp_ble_mesh_health_client_set_state_t set = {0};
    set.period_set.fast_period_divisor = divisor;
    return app_mesh_HealthSetCommon(dst, ESP_BLE_MESH_MODEL_OP_HEALTH_PERIOD_SET, &set);
}

int32_t app_mesh_SendHealthFaultClear(uint16_t dst, uint16_t company_id)
{
    esp_ble_mesh_health_client_set_state_t set = {0};
    set.fault_clear.company_id = company_id;
    return app_mesh_HealthSetCommon(dst, ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_CLEAR, &set);
}

int32_t app_mesh_SendHealthFaultTest(uint16_t dst, uint16_t company_id, uint8_t test_id)
{
    esp_ble_mesh_health_client_set_state_t set = {0};
    set.fault_test.company_id = company_id;
    set.fault_test.test_id = test_id;
    return app_mesh_HealthSetCommon(dst, ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_TEST, &set);
}

/* ============================================================
 *  app_mesh_Init
 * ============================================================ */
int32_t app_mesh_Init(void)
{
    if (g_bMeshInitialized) return ERROR_CODE_OK;

    g_hMeshMutex = xSemaphoreCreateMutex();
    if (!g_hMeshMutex) return ERROR_CODE_NO_MEMORY;

    g_hMeshEvtQ = xQueueCreate(DF_MESH_EVT_QUEUE_LEN, sizeof(st_mesh_evt_t));
    if (!g_hMeshEvtQ) return ERROR_CODE_NO_MEMORY;

    memset(&g_stInfo, 0, sizeof(g_stInfo));
    g_stInfo.u16NetIdx = DF_MESH_HOMI_NET_KEY_IDX;
    g_stInfo.u16AppIdx = DF_MESH_HOMI_APP_KEY_IDX_0;

    app_mesh_LoadConfig();

    if (xTaskCreate(app_mesh_Task, "mesh_task",
                    DF_MESH_TASK_STACK, NULL,
                    DF_MESH_TASK_PRIO, &g_hMeshTask) != pdPASS) {
        return ERROR_CODE_FAIL;
    }

    uint8_t au8Mac[6] = {0};
    esp_read_mac(au8Mac, ESP_MAC_BT);
    memcpy(&g_au8DevUuid[2], au8Mac, sizeof(au8Mac));
    LREP(TAG, "Device UUID MAC: %02X:%02X:%02X:%02X:%02X:%02X",
         au8Mac[0], au8Mac[1], au8Mac[2], au8Mac[3], au8Mac[4], au8Mac[5]);

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

    /* ---- Đăng ký Callbacks (Client + Server) ---- */
    esp_ble_mesh_register_prov_callback(app_mesh_ProvCb);
    esp_ble_mesh_register_config_server_callback(app_mesh_ConfigServerCb);
    esp_ble_mesh_register_generic_client_callback(app_mesh_GenericClientCb);
    esp_ble_mesh_register_light_client_callback(app_mesh_LightingClientCb);
    esp_ble_mesh_register_health_client_callback(app_mesh_HealthClientCb);
    esp_ble_mesh_register_custom_model_callback(app_mesh_CustomModelCb);

    char acName[32];
    snprintf(acName, sizeof(acName), "%s%02X%02X",
             DF_MESH_DEVICE_NAME_PREFIX, au8Mac[4], au8Mac[5]);

    err = esp_ble_mesh_init(&g_stProv, &g_stComp);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "esp_ble_mesh_init fail: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    esp_ble_mesh_set_unprovisioned_device_name(acName);

    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "prov_enable fail: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    g_bMeshInitialized = true;

    bool bProv = esp_ble_mesh_node_is_provisioned();
    LREP(TAG, "BLE Mesh khoi tao OK (Client Mode). Name=%s Prov=%s", acName, bProv ? "YES" : "NO");

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
    printf("   THONG TIN BLE MESH GATEWAY (HEALTH RX)\n");
    printf("========================================\n");
    printf("Trang thai     : %s\n", bProv ? "DA PROVISION" : "CHUA PROVISION");
    printf("Unicast Addr   : 0x%04X\n", addr);
    printf("NetIdx / AppIdx: 0x%04X / 0x%04X\n", snap.u16NetIdx, snap.u16AppIdx);
    printf("GatewayJoined  : %s\n", g_bGatewayJoined ? "YES" : "NO");
    printf("AppKey Mask    : 0x%02X (7=OK)\n", g_u8AppKeyReadyMask);
    printf("Group Sub'd    : 0x%04X\n", snap.u16Group);
    printf("Last OnOff RX  : %s\n", snap.u8OnOffState ? "ON" : "OFF");
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
