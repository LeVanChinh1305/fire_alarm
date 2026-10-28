#include "app_blemesh.h"
#include "app_config.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "esp_mac.h"
#include <string.h>

static const char *TAG = "APP_MESH";

// ==================== Biến toàn cục quản lý mesh ===============
static bool     g_bMeshInitialized  = false;
static bool     g_bProvisioned      = false;
static uint16_t g_u16NetIdx         = 0;        // Chỉ số mạng (Network Key Index) -> Quản lý khóa bảo mật lớp mạng. Mỗi mạng BLE Mesh có một NetKey riêng, và g_u16NetIdx lưu vị trí/chỉ số của khóa này để xác thực bản tin.
static uint16_t g_u16AppIdx         = 0;        // Chỉ số ứng dụng (Application Key Index) -> Quản lý khóa bảo mật lớp ứng dụng (AppKey). Dùng để mã hóa/giải mã các bản tin điều khiển cụ thể
static uint16_t g_u16Unicast        = 0;        // Địa chỉ Unicast (Unicast Address) của node -> Địa chỉ định danh duy nhất (16-bit) của thiết bị này trong mạng BLE Mesh. Khi khởi tạo mặc định là 0, sau khi Provision thành công, bộ quản lý mạng sẽ cấp cho thiết bị một địa chỉ cụ thể (ví dụ: 0x0002).
static uint8_t  g_u8OnOffState      = 0;        // Trạng thái OnOff hiện tại của Server

// =================== UUID của thiết bị (16 byte) ================
static uint8_t g_au8DevUuid[16] = {
    0x32, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ====== Định nghĩa cấu trúc Composition Data và Provisioning Data của một thiết bị BLE Mesh Node trong ESP-IDF =======
static esp_ble_mesh_cfg_srv_t g_stCfgSrv = {
    .relay              = ESP_BLE_MESH_RELAY_DISABLED, 
    .beacon             = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy         = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy         = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .friend_state       = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,        // Tắt tính năng Friend Node
    .default_ttl        = 7,                                        // Số bước chuyển tiếp tối đa 
    .net_transmit       = ESP_BLE_MESH_TRANSMIT(2, 20),             // Cấu hình phát lại gói tin lớp mạng: 2: Phát thêm 2 lần (tổng cộng phát 3 lần)/ 20: Khoảng thời gian nghỉ giữa các lần phát là 20 ms (đơn vị bước 10 ms).
    .relay_retransmit   = ESP_BLE_MESH_TRANSMIT(2, 20),             // Cấu hình phát lại nếu cờ Relay được bật về sau.
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stOnoffPub, 2 + 3, ROLE_NODE);


// OnOff Server
static esp_ble_mesh_gen_onoff_srv_t g_stOnoffSrv = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

static esp_ble_mesh_client_t g_stOnoffCli;      // OnOff Client

static esp_ble_mesh_model_t g_astRootModels[] = {                   // Mảng chứa danh sách các Model nằm trong Element chính (Root Element).
    ESP_BLE_MESH_MODEL_CFG_SRV(&g_stCfgSrv),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&g_stOnoffPub, &g_stOnoffSrv),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&g_stOnoffPub, &g_stOnoffCli),
};

static esp_ble_mesh_model_t g_astVndModels[] = {
    // Hiện tại chưa có Vendor Model
};

static esp_ble_mesh_elem_t g_astElements[] = {                      // Mảng chứa danh sách các Element (thành phần logic) của thiết bị
    ESP_BLE_MESH_ELEMENT(0, g_astRootModels, g_astVndModels),
    // 0: Chỉ số vị trí (Loc Location / Unspecified)-----g_astRootModels: Danh sách các SIG Models nằm trong Element này.
};

static esp_ble_mesh_comp_t g_stComp = {                             // Cấu trúc tổng hợp dữ liệu cấu thành Composition Data của thiết bị.
    .cid           = 0x02E5,                                        // Mã ID nhà sản xuất (Company ID) do Bluetooth SIG cấp: 0x02E5 là mã định danh chính thức của Espressif Systems.
    .element_count = ARRAY_SIZE(g_astElements),                     // Tự động đếm số lượng Element có trong mảng
    .elements      = g_astElements,                                 // Trỏ đến danh sách các Element đã định nghĩa.
};

static esp_ble_mesh_prov_t g_stProv = {                             // Cấu hình thông tin phục vụ quá trình Provisioning
    .uuid = g_au8DevUuid,
};

//=================================== Callback ================
static void app_mesh_ProvCb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        LREP(TAG, "PROV_REGISTER_COMP, err_code %d", param->prov_register_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        LREP(TAG, "NODE_PROV_ENABLE_COMP, err_code %d", param->node_prov_enable_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        LREP(TAG, "PROV_LINK_OPEN, bearer %s",
             param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        LREP(TAG, "PROV_LINK_CLOSE, bearer %s",
             param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        LREP(TAG, "=== PROVISION Hoàn Tất ===");
        LREP(TAG, "  NetIdx  : 0x%04x", param->node_prov_complete.net_idx);
        LREP(TAG, "  Unicast : 0x%04x", param->node_prov_complete.addr);
        LREP(TAG, "  Flags   : 0x%02x", param->node_prov_complete.flags);
        LREP(TAG, "  IV Index: 0x%08lx", param->node_prov_complete.iv_index);

        g_bProvisioned = true;
        g_u16NetIdx    = param->node_prov_complete.net_idx;
        g_u16Unicast   = param->node_prov_complete.addr;
        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        LREP(TAG, "NODE_PROV_RESET");
        g_bProvisioned = false;
        g_u16Unicast   = 0;
        break;

    default:
        break;
    }
}

static void app_mesh_ConfigServerCb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            LREP(TAG, "APP_KEY_ADD: net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            g_u16AppIdx = param->value.state_change.appkey_add.app_idx;
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

static void app_mesh_GenericServerCb(esp_ble_mesh_generic_server_cb_event_t event, esp_ble_mesh_generic_server_cb_param_t *param)
{
    if (param == NULL) return;

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {

            g_u8OnOffState = param->value.state_change.onoff_set.onoff;
            LREP(TAG, "OnOff Server thay đổi trạng thái -> %s", g_u8OnOffState ? "ON" : "OFF");
        }
        break;

    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            esp_ble_mesh_gen_onoff_srv_t *srv = param->model->user_data;
            esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        }
        break;

    default:
        break;
    }
}

int32_t app_mesh_Init(void){
    esp_err_t err;
    // lấy địa chỉ mac thiết bị để tạo UUID duy nhất
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(&g_au8DevUuid[2], mac, 6);
    LREP(TAG, "Device UUID (MAC): %02X : %02X : %02X : %02X : %02X : %02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Khởi tạo bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if(err){
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

    // đăng ký callback 
    err = esp_ble_mesh_register_prov_callback(app_mesh_ProvCb);                     // Đăng ký hàm app_mesh_ProvCb callback chuyên quản lý luồng Provisioning: Sự kiện bắt đầu / kết thúc Provisioning ----- Sự kiện gán địa chỉ Unicast ---- Sự kiện Reset mạng
    if (err) {
        LREP_ERROR(TAG, "register_prov_callback failed");
        return ERROR_CODE_FAIL;
    }
    err = esp_ble_mesh_register_config_server_callback(app_mesh_ConfigServerCb);    // Đăng ký hàm app_mesh_ConfigServerCb callback để xử lý các sự kiện liên quan đến Configuration Server Model (Model cấu hình hệ thống chuẩn SIG Mesh): Thêm / Gán AppKey ----- Đăng ký nhóm / Subscription ----- Cài đặt tham số truyền nhận
    if (err) {
        LREP_ERROR(TAG, "register_config_server_callback failed");
        return ERROR_CODE_FAIL;
    }
    err = esp_ble_mesh_register_generic_server_callback(app_mesh_GenericServerCb);  // đăng ký Config Server callback
    if (err) {
        LREP_ERROR(TAG, "register_generic_server_callback failed");
        return ERROR_CODE_FAIL;
    }

    /* khởi tạo Mesh stack: Nạp toàn bộ cấu hình vào BLE Mesh Stack của ESP-IDF và bắt đầu phân bổ tài nguyên cho mạng Mesh. Bao gồm: 
        &g_stProv (esp_ble_mesh_prov_t): Chứa thông tin cấu hình Provisioning của thiết bị (bao gồm UUID thiết bị g_au8DevUuid, các phương thức xác thực OOB, địa chỉ gán ban đầu, các hàm callback nhận thông báo,...
        &g_stComp (esp_ble_mesh_comp_t): Cấu trúc Composition Data chứa thông tin về phần cứng của thiết bị (Company ID, Product ID, số lượng Element, và danh sách các Model chuẩn SIG như Config Server, OnOff Server,... mà Node đó hỗ trợ). */
    err = esp_ble_mesh_init(&g_stProv, &g_stComp);
    if (err) {
        LREP_ERROR(TAG, "esp_ble_mesh_init failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    /* Bật Provisioning: Bật tính năng phát quảng cáo (Advertising) ở chế độ Unprovisioned Device. Tham số bearer truyền vào:
        ESP_BLE_MESH_PROV_ADV (PB-ADV): Cho phép nạp mạng thông qua kênh truyền gói tin quảng cáo Bluetooth
        ESP_BLE_MESH_PROV_GATT (PB-GATT): Cho phép nạp mạng thông qua kết nối Bluetooth Low Energy tiêu chuẩn   */
    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err) {
        LREP_ERROR(TAG, "node_prov_enable failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    g_bMeshInitialized = true;
    g_bProvisioned = esp_ble_mesh_node_is_provisioned();

    LREP(TAG, "BLE Mesh Node khởi tạo thành công. Provisioned: %s", g_bProvisioned ? "YES" : "NO");

    return ERROR_CODE_OK;

}

// ========================= Các API hỗ trợ khác =======================
bool app_mesh_IsProvisioned(void){
    return g_bProvisioned || (g_bMeshInitialized && esp_ble_mesh_node_is_provisioned());
}

void app_mesh_PrintStatus(void)
{
    printf("\n=== Trạng Thái Bluetooth Mesh ===\n");
    printf("Initialized : %s\n"     , g_bMeshInitialized ? "YES" : "NO");
    printf("Provisioned : %s\n"     , app_mesh_IsProvisioned() ? "YES" : "NO");
    printf("Unicast     : 0x%04X\n" , g_u16Unicast);
    printf("NetIdx      : 0x%04X\n" , g_u16NetIdx);
    printf("AppIdx      : 0x%04X\n" , g_u16AppIdx);
    printf("OnOff State : %s\n", g_u8OnOffState ? "ON" : "OFF");
    printf("===========================\n\n");
}

int32_t app_mesh_Reset(void)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

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

int32_t app_mesh_SendOnOff(uint16_t u16DstAddr, uint8_t u8OnOff)
{
    if (!g_bMeshInitialized || !app_mesh_IsProvisioned()) {
        LREP_ERROR(TAG, "Mesh chưa sẵn sàng hoặc chưa provision");
        return ERROR_CODE_FAIL;
    }

    static uint8_t tid = 0;

    esp_ble_mesh_generic_client_set_state_t set = {0};
    set.onoff_set.op_en = false;
    set.onoff_set.onoff = u8OnOff ? 1 : 0;
    set.onoff_set.tid   = tid++;

    esp_ble_mesh_client_common_param_t common = {
        .opcode         = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
        .model          = &g_astRootModels[2],   // OnOff Client nằm ở index 2
        .ctx.net_idx    = g_u16NetIdx,
        .ctx.app_idx    = g_u16AppIdx,
        .ctx.addr       = u16DstAddr,
        .ctx.send_ttl   = 7,
        .msg_timeout    = 0,
    };

    esp_err_t err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Send OnOff failed: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    LREP(TAG, "Đã gửi OnOff %s tới 0x%04X", u8OnOff ? "ON" : "OFF", u16DstAddr);
    return ERROR_CODE_OK;
}

uint8_t app_mesh_GetOnOffState(void)
{
    return g_u8OnOffState;
}