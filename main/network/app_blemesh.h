#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "app_common.h"
#include "app_mesh_vendor.h"

/* ================== Khởi tạo / Reset ================== */

/**
 * @brief Khởi tạo BLE Mesh Node (15 model: Config/Health/Generic/Lighting/Vendor)
 *        Tạo Queue + Task xử lý sự kiện, khởi tạo Bluetooth, đăng ký callback.
 */
int32_t app_mesh_Init(void);

/**
 * @brief Reset node về trạng thái Unprovisioned, xoá cấu hình mesh
 */
int32_t app_mesh_Reset(void);

/* ================== Trạng thái ================== */

/** @brief Kiểm tra node đã được provision chưa */
bool app_mesh_IsProvisioned(void);

/** @brief Lấy trạng thái OnOff hiện tại của Server */
uint8_t app_mesh_GetOnOffState(void);

/* ================== Gateway Join ================== */

/**
 * @brief Tự động thêm 3 AppKey HOMI, bind tất cả model, subscribe Group 0xC000.
 *        Hoạt động bất đồng bộ qua Task riêng.
 * @return ERROR_CODE_OK: đã khởi động task
 *         ERROR_CODE_BUSY: đang chạy
 *         ERROR_CODE_FAIL: chưa provision hoặc lỗi
 */
int32_t app_mesh_GatewayJoin(void);

/** @brief Kiểm tra Gateway đã join HOMI thành công chưa */
bool app_mesh_GetGatewayJoined(void);

/* ================== Console / Debug ================== */

/**
 * @brief In trạng thái Mesh (Provisioned, Unicast, NetIdx, AppIdx, OnOff...)
 */
void app_mesh_PrintStatus(void);

/**
 * @brief In chi tiết tất cả model: AppKey đã bind, Group đã subscribe.
 *        Dùng cho lệnh console "meshmodels".
 */
void app_mesh_PrintModels(void);


int32_t app_mesh_HealthFaultGet(uint16_t dst);