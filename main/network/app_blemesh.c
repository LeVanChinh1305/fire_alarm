#include "app_blemesh.h"
#include "app_config.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_health_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"

#include "esp_mac.h"
#include <string.h>

static const char *TAG = "APP_MESH";

// ==================== Biến toàn cục quản lý mesh ====================
static bool g_bMeshInitialized = false;
static bool g_bProvisioned = false;
static uint16_t g_u16NetIdx = 0;      // Chỉ số mạng (Network Key Index)
static uint16_t g_u16AppIdx = 0;      // Chỉ số ứng dụng (AppKey Index) của AppKey cuối cùng được thêm
static uint16_t g_u16Unicast = 0;     // Địa chỉ Unicast của node trong mạng BLE Mesh (cấp sau khi Provision)
static uint8_t g_u8OnOffState = 0;    // Trạng thái OnOff hiện tại của Server

// ==================== UUID của thiết bị (16 byte) ====================
static uint8_t g_au8DevUuid[16] = {
    0x32, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ==================== Config Server ====================
static esp_ble_mesh_cfg_srv_t g_stCfgSrv = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED, // Tắt tính năng Friend Node
    .default_ttl = 7,                                  // Số bước chuyển tiếp tối đa
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),       // Phát thêm 2 lần (tổng 3 lần), cách nhau 20 ms
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),  // Cấu hình phát lại nếu Relay được bật về sau
};

// ==================== 15 MODEL (giống nút bấm HOMI 0x0065) ====================
// Element 0 (13 model): Config Srv, Health Srv, Health Cli, Generic OnOff Srv,
// Generic Level Srv, Generic Default Transition Time Srv, Generic Power OnOff Srv,
// Generic Power OnOff Setup Srv, Light Lightness Srv, Light Lightness Setup Srv,
// Light CTL Srv, Light CTL Setup Srv, Vendor 0x8888/0x0000
// Element 1 (2 model): Generic Level Srv, Light CTL Temperature Srv

#define DF_SRV_RSP_AUTO \
    { .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP, \
      .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP }

/* ---------- Publication: mỗi model một buffer riêng ---------- */
ESP_BLE_MESH_HEALTH_PUB_DEFINE(g_stHealthPub, 8, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stOnoffPub, 2 + 3, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLevelPub, 2 + 5, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stDttPub, 2 + 1, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stPwrPub, 2 + 1, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stPwrSetupPub, 2 + 1, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLightnessPub, 2 + 5, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLightnessSetupPub, 2 + 5, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlPub, 2 + 9, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlSetupPub, 2 + 9, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stLevel2Pub, 2 + 5, ROLE_NODE);  // element 1
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stCtlTempPub, 2 + 9, ROLE_NODE); // element 1

/* ---------- Health ---------- */
/* Health Server bắt buộc có ít nhất 1 test ID, không có -> esp_ble_mesh_init lỗi -22 */
static const uint8_t g_au8HealthTestIds[] = { 0x00 };
static esp_ble_mesh_health_srv_t g_stHealthSrv = {
    .health_test.id_count = ARRAY_SIZE(g_au8HealthTestIds),
    .health_test.test_ids = g_au8HealthTestIds,
};
static esp_ble_mesh_client_t g_stHealthCli;

/* ---------- State dùng chung giữa server và setup server (theo spec/API) ---------- */
static esp_ble_mesh_gen_onpowerup_state_t g_stOnPowerUpState;

static esp_ble_mesh_light_lightness_state_t g_stLightnessState = {
    .lightness_range_min = 1,
    .lightness_range_max = 0xFFFF,
    .lightness_default = 0xFFFF,
};

static esp_ble_mesh_light_ctl_state_t g_stCtlState = {
    .lightness_default = 0xFFFF,
    .temperature = 4000,
    .temperature_default = 4000,
    .temperature_range_min = 800,
    .temperature_range_max = 20000,
};

/* ---------- Generic servers ---------- */
static esp_ble_mesh_gen_onoff_srv_t g_stOnoffSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};

static esp_ble_mesh_gen_level_srv_t g_stLevelSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};

static esp_ble_mesh_gen_level_srv_t g_stLevel2Srv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};

static esp_ble_mesh_gen_def_trans_time_srv_t g_stDttSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO
};

static esp_ble_mesh_gen_power_onoff_srv_t g_stPwrSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stOnPowerUpState
};

static esp_ble_mesh_gen_power_onoff_setup_srv_t g_stPwrSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stOnPowerUpState
};

/* ---------- Lighting servers ---------- */
static esp_ble_mesh_light_lightness_srv_t g_stLightnessSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stLightnessState
};

static esp_ble_mesh_light_lightness_setup_srv_t g_stLightnessSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stLightnessState
};

static esp_ble_mesh_light_ctl_srv_t g_stCtlSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stCtlState
};

static esp_ble_mesh_light_ctl_setup_srv_t g_stCtlSetupSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stCtlState
};

static esp_ble_mesh_light_ctl_temp_srv_t g_stCtlTempSrv = {
    .rsp_ctrl = DF_SRV_RSP_AUTO,
    .state = &g_stCtlState
};

// ==================== Element 0: SIG models ====================
static esp_ble_mesh_model_t g_astRootModels[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&g_stCfgSrv),
    ESP_BLE_MESH_MODEL_HEALTH_SRV(&g_stHealthSrv, &g_stHealthPub),
    ESP_BLE_MESH_MODEL_HEALTH_CLI(&g_stHealthCli),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&g_stOnoffPub, &g_stOnoffSrv),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&g_stLevelPub, &g_stLevelSrv),
    ESP_BLE_MESH_MODEL_GEN_DEF_TRANS_TIME_SRV(&g_stDttPub, &g_stDttSrv),
    ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SRV(&g_stPwrPub, &g_stPwrSrv),
    ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SETUP_SRV(&g_stPwrSetupPub, &g_stPwrSetupSrv),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SRV(&g_stLightnessPub, &g_stLightnessSrv),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SETUP_SRV(&g_stLightnessSetupPub, &g_stLightnessSetupSrv),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_SRV(&g_stCtlPub, &g_stCtlSrv),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_SETUP_SRV(&g_stCtlSetupPub, &g_stCtlSetupSrv),
};

// ==================== Element 1: SIG models ====================
static esp_ble_mesh_model_t g_astSecModels[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&g_stLevel2Pub, &g_stLevel2Srv),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_TEMP_SRV(&g_stCtlTempPub, &g_stCtlTempSrv),
};

// ==================== Vendor model (element 0) ====================
// Đăng ký toàn bộ 64 opcode (0x00-0x3F) làm tấm lưới hứng mọi bản tin vendor
#define VND_OP(n) ESP_BLE_MESH_MODEL_OP(DF_VND_OPCODE(n), 0)

static esp_ble_mesh_model_op_t g_astVndServerOp[] = {
    VND_OP(0x00),
    VND_OP(0x01),
    VND_OP(0x02),
    VND_OP(0x03),
    VND_OP(0x04),
    VND_OP(0x05),
    VND_OP(0x06),
    VND_OP(0x07),
    VND_OP(0x08),
    VND_OP(0x09),
    VND_OP(0x0A),
    VND_OP(0x0B),
    VND_OP(0x0C),
    VND_OP(0x0D),
    VND_OP(0x0E),
    VND_OP(0x0F),

    VND_OP(0x10),
    VND_OP(0x11),
    VND_OP(0x12),
    VND_OP(0x13),
    VND_OP(0x14),
    VND_OP(0x15),
    VND_OP(0x16),
    VND_OP(0x17),
    VND_OP(0x18),
    VND_OP(0x19),
    VND_OP(0x1A),
    VND_OP(0x1B),
    VND_OP(0x1C),
    VND_OP(0x1D),
    VND_OP(0x1E),
    VND_OP(0x1F),

    VND_OP(0x20),
    VND_OP(0x21),
    VND_OP(0x22),
    VND_OP(0x23),
    VND_OP(0x24),
    VND_OP(0x25),
    VND_OP(0x26),
    VND_OP(0x27),
    VND_OP(0x28),
    VND_OP(0x29),
    VND_OP(0x2A),
    VND_OP(0x2B),
    VND_OP(0x2C),
    VND_OP(0x2D),
    VND_OP(0x2E),
    VND_OP(0x2F),

    VND_OP(0x30),
    VND_OP(0x31),
    VND_OP(0x32),
    VND_OP(0x33),
    VND_OP(0x34),
    VND_OP(0x35),
    VND_OP(0x36),
    VND_OP(0x37),
    VND_OP(0x38),
    VND_OP(0x39),
    VND_OP(0x3A),
    VND_OP(0x3B),
    VND_OP(0x3C),
    VND_OP(0x3D),
    VND_OP(0x3E),
    VND_OP(0x3F),

    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t g_astVndModels[] = {
    ESP_BLE_MESH_VENDOR_MODEL(
        DF_MESH_VND_COMPANY_ID,
        DF_MESH_VND_MODEL_ID_SERVER,
        g_astVndServerOp,
        NULL,
        NULL
    ),
};

// ==================== Elements + Composition ====================
static esp_ble_mesh_elem_t g_astElements[] = {
    ESP_BLE_MESH_ELEMENT(0, g_astRootModels, g_astVndModels),
    ESP_BLE_MESH_ELEMENT(0, g_astSecModels, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t g_stComp = {
    .cid = 0x02E5, // Company ID Espressif
    .element_count = ARRAY_SIZE(g_astElements),
    .elements = g_astElements,
};

static esp_ble_mesh_prov_t g_stProv = {
    .uuid = g_au8DevUuid,
};

// ==================== Gateway Join: cấu hình ====================
#define DF_HOMI_APPKEY_IDX 0x0000 // Kiểm tra lại đúng AppKey Index của mạng HOMI
#define DF_HOMI_GROUP_ADDR 0xC000

/* TODO: điền 16 byte AppKey HOMI lấy từ code gốc */
static const uint8_t g_au8HomiAppKey[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

typedef struct {
    uint8_t u8Elem;       // 0 hoặc 1
    uint16_t u16Cid;      // ESP_BLE_MESH_CID_NVAL cho SIG model
    uint16_t u16ModelId;
    bool bSub;            // có subscribe Group 0xC000 hay không
} df_join_item_t;

/* 14 model (trừ Config Server vì Config Server dùng DevKey, không bind AppKey) */
static const df_join_item_t g_astJoin[] = {
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_HEALTH_CLI, false },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_DEF_TRANS_TIME_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_POWER_ONOFF_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_POWER_ONOFF_SETUP_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SRV, true },
    { 0, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SETUP_SRV, true },
    { 0, DF_MESH_VND_COMPANY_ID, DF_MESH_VND_MODEL_ID_SERVER, true },
    { 1, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV, true },
    { 1, ESP_BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_TEMP_SRV, true },
};

static SemaphoreHandle_t g_semAppKey = NULL;
static volatile bool g_bJoinRunning = false;

// ==================== Callback ====================
static void app_mesh_ProvCb(
    esp_ble_mesh_prov_cb_event_t event,
    esp_ble_mesh_prov_cb_param_t *param
) {
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        LREP(TAG, "PROV_REGISTER_COMP, err_code %d",
             param->prov_register_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        LREP(TAG, "NODE_PROV_ENABLE_COMP, err_code %d",
             param->node_prov_enable_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        LREP(TAG, "PROV_LINK_OPEN, bearer %s",
             param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV
                 ? "PB-ADV"
                 : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        LREP(TAG, "PROV_LINK_CLOSE, bearer %s",
             param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV
                 ? "PB-ADV"
                 : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        LREP(TAG, "=== PROVISION Hoàn Tất ===");
        LREP(TAG, "  NetIdx  : 0x%04x", param->node_prov_complete.net_idx);
        LREP(TAG, "  Unicast : 0x%04x", param->node_prov_complete.addr);
        LREP(TAG, "  Flags   : 0x%02x", param->node_prov_complete.flags);
        LREP(TAG, "  IV Index: 0x%08lx",
             (unsigned long)param->node_prov_complete.iv_index);

        g_bProvisioned = true;
        g_u16NetIdx = param->node_prov_complete.net_idx;
        g_u16Unicast = param->node_prov_complete.addr;

        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        LREP(TAG, "NODE_PROV_RESET");
        g_bProvisioned = false;
        g_u16Unicast = 0;
        break;

    case ESP_BLE_MESH_NODE_ADD_LOCAL_APP_KEY_COMP_EVT:
        LREP(TAG, "[JOIN] ADD_APPKEY err %d, net 0x%04x, app 0x%04x",
             param->node_add_app_key_comp.err_code,
             param->node_add_app_key_comp.net_idx,
             param->node_add_app_key_comp.app_idx);
        if (g_semAppKey) {
            xSemaphoreGive(g_semAppKey);
        }
        break;

    case ESP_BLE_MESH_NODE_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        LREP(TAG, "[JOIN] BIND_COMP err %d, elem 0x%04x, cid 0x%04x, mod 0x%04x",
             param->node_bind_app_key_to_model_comp.err_code,
             param->node_bind_app_key_to_model_comp.element_addr,
             param->node_bind_app_key_to_model_comp.company_id,
             param->node_bind_app_key_to_model_comp.model_id);
        break;

    case ESP_BLE_MESH_MODEL_SUBSCRIBE_GROUP_ADDR_COMP_EVT:
        LREP(TAG, "[JOIN] SUB_COMP err %d, elem 0x%04x, mod 0x%04x, grp 0x%04x",
             param->model_sub_group_addr_comp.err_code,
             param->model_sub_group_addr_comp.element_addr,
             param->model_sub_group_addr_comp.model_id,
             param->model_sub_group_addr_comp.group_addr);
        break;

    default:
        break;
    }
}

static void app_mesh_ConfigServerCb(
    esp_ble_mesh_cfg_server_cb_event_t event,
    esp_ble_mesh_cfg_server_cb_param_t *param
) {
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            LREP(TAG, "APP_KEY_ADD: net_idx 0x%04x, app_idx 0x%04x",
                 param->value.state_change.appkey_add.net_idx,
                 param->value.state_change.appkey_add.app_idx);
            g_u16AppIdx = param->value.state_change.appkey_add.app_idx;
            app_mesh_GatewayJoin(); // Tu dong bind tat ca model sau khi nRF Mesh add AppKey
            break;

        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            LREP(TAG, "MODEL_APP_BIND: elem 0x%04x, app_idx 0x%04x, mod_id 0x%04x",
                 param->value.state_change.mod_app_bind.element_addr,
                 param->value.state_change.mod_app_bind.app_idx,
                 param->value.state_change.mod_app_bind.model_id);
            break;

        default:
            break;
        }
    }
}

static void app_mesh_GenericServerCb(
    esp_ble_mesh_generic_server_cb_event_t event,
    esp_ble_mesh_generic_server_cb_param_t *param
) {
    if (param == NULL || event != ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT) {
        return;
    }

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        g_u8OnOffState = param->value.state_change.onoff_set.onoff;
        LREP(TAG, "OnOff -> %s", g_u8OnOffState ? "ON" : "OFF");
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK:
        LREP(TAG, "Level[elem %d] -> %d",
             param->model->element_idx,
             param->value.state_change.level_set.level);
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_DEF_TRANS_TIME_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_DEF_TRANS_TIME_SET_UNACK:
        LREP(TAG, "DefTransTime -> 0x%02x",
             param->value.state_change.def_trans_time_set.trans_time);
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_SET_UNACK:
        LREP(TAG, "OnPowerUp -> %u",
             param->value.state_change.onpowerup_set.onpowerup);
        break;

    default:
        LREP(TAG, "Generic state change, op 0x%08lx",
             (unsigned long)param->ctx.recv_op);
        break;
    }
}

static void app_mesh_LightingServerCb(
    esp_ble_mesh_lighting_server_cb_event_t event,
    esp_ble_mesh_lighting_server_cb_param_t *param
) {
    if (param == NULL || event != ESP_BLE_MESH_LIGHTING_SERVER_STATE_CHANGE_EVT) {
        return;
    }

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET:
    case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK:
        LREP(TAG, "Lightness -> %u",
             param->value.state_change.lightness_set.lightness);
        break;

    case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET:
    case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET_UNACK:
        LREP(TAG, "CTL -> lightness %u, temp %u, duv %d",
             param->value.state_change.ctl_set.lightness,
             param->value.state_change.ctl_set.temperature,
             param->value.state_change.ctl_set.delta_uv);
        break;

    case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET:
    case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET_UNACK:
        LREP(TAG, "CTL Temp -> temp %u, duv %d",
             param->value.state_change.ctl_temp_set.temperature,
             param->value.state_change.ctl_temp_set.delta_uv);
        break;

    default:
        LREP(TAG, "Lighting state change, op 0x%08lx",
             (unsigned long)param->ctx.recv_op);
        break;
    }
}

/* ---------- Vendor model callback: hứng và log raw tất cả bản tin ---------- */
static void app_mesh_CustomModelCb(
    esp_ble_mesh_model_cb_event_t event,
    esp_ble_mesh_model_cb_param_t *param
) {
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT) {
        return;
    }

    LREP(TAG, "Vendor RX: opcode=0x%08lx, len=%d, src=0x%04x, dst=0x%04x",
         (unsigned long)param->model_operation.opcode,
         param->model_operation.length,
         param->model_operation.ctx->addr,
         param->model_operation.ctx->recv_dst);

    /* In raw payload (hex) để quan sát định dạng bản tin */
    if (param->model_operation.msg && param->model_operation.length > 0) {
        printf("  Payload (%d bytes):", param->model_operation.length);
        for (int i = 0; i < param->model_operation.length; i++) {
            printf(" %02X", param->model_operation.msg[i]);
        }
        printf("\n");
    }
}

int32_t app_mesh_Init(void)
{
    esp_err_t err;

    // lấy địa chỉ mac thiết bị để tạo UUID duy nhất
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(&g_au8DevUuid[2], mac, 6);
    LREP(TAG, "Device UUID (MAC): %02X : %02X : %02X : %02X : %02X : %02X",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Khởi tạo bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err) {
        LREP_ERROR(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err) {
        LREP_ERROR(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    // Khởi tạo bluedroid
    err = esp_bluedroid_init();
    if (err) {
        LREP_ERROR(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    err = esp_bluedroid_enable();
    if (err) {
        LREP_ERROR(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    // đăng ký callback (phải đăng ký trước esp_ble_mesh_init)
    err = esp_ble_mesh_register_prov_callback(app_mesh_ProvCb);
    if (err) {
        LREP_ERROR(TAG, "register_prov_callback failed");
        return ERROR_CODE_FAIL;
    }

    err = esp_ble_mesh_register_config_server_callback(app_mesh_ConfigServerCb);
    if (err) {
        LREP_ERROR(TAG, "register_config_server_callback failed");
        return ERROR_CODE_FAIL;
    }

    err = esp_ble_mesh_register_generic_server_callback(app_mesh_GenericServerCb);
    if (err) {
        LREP_ERROR(TAG, "register_generic_server_callback failed");
        return ERROR_CODE_FAIL;
    }

    err = esp_ble_mesh_register_lighting_server_callback(app_mesh_LightingServerCb);
    if (err) {
        LREP_ERROR(TAG, "register_lighting_server_callback failed");
        return ERROR_CODE_FAIL;
    }

    err = esp_ble_mesh_register_custom_model_callback(app_mesh_CustomModelCb);
    if (err) {
        LREP_ERROR(TAG, "register_custom_model_callback failed");
        return ERROR_CODE_FAIL;
    }

    /* Khởi tạo Mesh stack với Provisioning data + Composition data (15 model, 2 element) */
    err = esp_ble_mesh_init(&g_stProv, &g_stComp);
    if (err) {
        LREP_ERROR(TAG, "esp_ble_mesh_init failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    /* Bật Provisioning qua cả PB-ADV và PB-GATT (nRF Mesh trên điện thoại dùng PB-GATT) */
    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(
            ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT
        )
    );
    if (err) {
        LREP_ERROR(TAG, "node_prov_enable failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    g_bMeshInitialized = true;
    g_bProvisioned = esp_ble_mesh_node_is_provisioned();

    LREP(TAG, "BLE Mesh Node khởi tạo thành công. Provisioned: %s",
         g_bProvisioned ? "YES" : "NO");


    /* GatewayJoin KHONG tu chay -- de provisioner (nRF Mesh hoac HOMI gateway)
     * tu gui Config AppKey Add / Model App Bind / Model Subscription Add.
     * Goi app_mesh_GatewayJoin() thu cong (lenh 'meshjoin') khi can join HOMI. */
    return ERROR_CODE_OK;
}

// ==================== Gateway Join ====================
static void app_mesh_JoinTask(void *pvArg)
{
    vTaskDelay(pdMS_TO_TICKS(500)); // chờ link provisioning đóng hẳn
    uint16_t u16Base = esp_ble_mesh_get_primary_element_address();
    LREP(TAG, "[JOIN] bắt đầu, element gốc 0x%04x", u16Base);

    /* 1. Thêm AppKey HOMI (đã tồn tại thì lỗi này vô hại) */
    xSemaphoreTake(g_semAppKey, 0);
    esp_err_t err = esp_ble_mesh_node_add_local_app_key(
        g_au8HomiAppKey,
        g_u16NetIdx,
        DF_HOMI_APPKEY_IDX
    );

    if (err == ESP_OK) {
        xSemaphoreTake(g_semAppKey, pdMS_TO_TICKS(2000));
    } else {
        LREP_WARNING(TAG, "[JOIN] add_local_app_key: %s", esp_err_to_name(err));
    }

    /* 2. Bind AppKey rồi subscribe Group 0xC000 cho từng model */
    for (size_t i = 0; i < ARRAY_SIZE(g_astJoin); i++) {
        uint16_t u16Elem = u16Base + g_astJoin[i].u8Elem;

        err = esp_ble_mesh_node_bind_app_key_to_local_model(
            u16Elem,
            g_astJoin[i].u16Cid,
            g_astJoin[i].u16ModelId,
            g_u16AppIdx
        );

        if (err != ESP_OK) {
            LREP_WARNING(TAG, "[JOIN] bind 0x%04x: %s",
                         g_astJoin[i].u16ModelId,
                         esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(30));

        if (g_astJoin[i].bSub) {
            err = esp_ble_mesh_model_subscribe_group_addr(
                u16Elem,
                g_astJoin[i].u16Cid,
                g_astJoin[i].u16ModelId,
                DF_HOMI_GROUP_ADDR
            );

            if (err != ESP_OK) {
                LREP_WARNING(TAG, "[JOIN] sub 0x%04x: %s",
                             g_astJoin[i].u16ModelId,
                             esp_err_to_name(err));
            }

            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

    LREP(TAG, "[JOIN] hoàn tất (xem các dòng BIND_COMP / SUB_COMP phía trên)");
    g_bJoinRunning = false;
    vTaskDelete(NULL);
}

int32_t app_mesh_GatewayJoin(void)
{
    if (!g_bMeshInitialized && !g_bProvisioned) {
        return ERROR_CODE_FAIL;
    }

    if (!app_mesh_IsProvisioned()) {
        return ERROR_CODE_FAIL;
    }

    if (g_bJoinRunning) {
        return ERROR_CODE_BUSY;
    }

    if (g_semAppKey == NULL) {
        g_semAppKey = xSemaphoreCreateBinary();
        if (g_semAppKey == NULL) {
            return ERROR_CODE_NO_MEMORY;
        }
    }

    g_bJoinRunning = true;
    if (xTaskCreate(app_mesh_JoinTask, "mesh_join", 4096, NULL, 5, NULL) != pdPASS) {
        g_bJoinRunning = false;
        return ERROR_CODE_NO_MEMORY;
    }

    return ERROR_CODE_OK;
}

// ==================== Các API hỗ trợ khác ====================
bool app_mesh_IsProvisioned(void)
{
    return g_bProvisioned ||
           (g_bMeshInitialized && esp_ble_mesh_node_is_provisioned());
}

void app_mesh_PrintStatus(void)
{
    printf("\n=== Trạng Thái Bluetooth Mesh ===\n");
    printf("Initialized : %s\n", g_bMeshInitialized ? "YES" : "NO");
    printf("Provisioned : %s\n", app_mesh_IsProvisioned() ? "YES" : "NO");
    printf("Unicast     : 0x%04X\n", g_u16Unicast);
    printf("NetIdx      : 0x%04X\n", g_u16NetIdx);
    printf("AppIdx      : 0x%04X\n", g_u16AppIdx);
    printf("OnOff State : %s\n", g_u8OnOffState ? "ON" : "OFF");
    printf("===========================\n\n");
}

static void app_mesh_PrintModelArr(
    const char *pcName,
    const esp_ble_mesh_model_t *pModels,
    size_t n,
    bool bVendor
) {
    for (size_t i = 0; i < n; i++) {
        uint16_t u16Id = bVendor ? pModels[i].vnd.model_id : pModels[i].model_id;
        uint16_t u16Cid = bVendor ? pModels[i].vnd.company_id : 0xFFFF;

        printf("%-6s cid 0x%04X id 0x%04X | key0 0x%04X | grp0 0x%04X\n",
               pcName,
               u16Cid,
               u16Id,
               pModels[i].keys[0],
               pModels[i].groups[0]);
    }
}

void app_mesh_PrintModels(void)
{
    printf("\n=== Models (key 0xFFFF = chưa bind, grp 0x0000/0xFFFF = chưa "
           "subscribe) ===\n");

    app_mesh_PrintModelArr(
        "elem0",
        g_astRootModels,
        ARRAY_SIZE(g_astRootModels),
        false
    );

    app_mesh_PrintModelArr(
        "elem0",
        g_astVndModels,
        ARRAY_SIZE(g_astVndModels),
        true
    );

    app_mesh_PrintModelArr(
        "elem1",
        g_astSecModels,
        ARRAY_SIZE(g_astSecModels),
        false
    );

    printf("\n");
}

int32_t app_mesh_Reset(void)
{
    if (!g_bMeshInitialized) {
        return ERROR_CODE_FAIL;
    }

    esp_err_t err = esp_ble_mesh_node_local_reset();
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "node_local_reset failed");
        return ERROR_CODE_FAIL;
    }

    g_bProvisioned = false;
    g_u16Unicast = 0;
    LREP(TAG, "Đã reset mesh về Unprovisioned");

    return ERROR_CODE_OK;
}

uint8_t app_mesh_GetOnOffState(void)
{
    return g_u8OnOffState;
}
