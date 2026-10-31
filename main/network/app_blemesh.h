#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "app_common.h"
#include "app_mesh_vendor.h"

/* ================== Khởi tạo / Reset ================== */

/**
 * @brief Khởi tạo BLE Mesh Node (Config/Health CLI+SRV, Generic CLI, Light CLI, Vendor SRV)
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

/** @brief Lấy trạng thái OnOff cache được từ Client Status */
uint8_t app_mesh_GetOnOffState(void);

/* ================== Gateway Join ================== */

/**
 * @brief Tự động thêm 3 AppKey HOMI, bind Health Client và subscribe Group 0xC000.
 *        Hoạt động bất đồng bộ qua Task riêng.
 * @return ERROR_CODE_OK: đã khởi động task
 *         ERROR_CODE_BUSY: đang chạy
 *         ERROR_CODE_FAIL: chưa provision hoặc lỗi
 */
int32_t app_mesh_GatewayJoin(void);

/** @brief Kiểm tra Gateway đã join HOMI thành công chưa */
bool app_mesh_GetGatewayJoined(void);

/* ================== Client Model Commands (Chủ động Get/Read) ================== */

/**
 * @brief Gửi lệnh Generic OnOff Get đến node unicast
 */
int32_t app_mesh_SendOnOffGet(uint16_t u16DstAddr);

/**
 * @brief Gửi lệnh Health Fault Get đến node unicast
 */
int32_t app_mesh_SendHealthFaultGet(uint16_t u16DstAddr, uint16_t u16CompanyId);

/**
 * @brief Gửi lệnh Health Attention Get đến node unicast
 */
int32_t app_mesh_SendHealthAttentionGet(uint16_t u16DstAddr);

/**
 * @brief Gửi lệnh Health Period Get đến node unicast
 */
int32_t app_mesh_SendHealthPeriodGet(uint16_t u16DstAddr);

/** @brief Health Attention Set (0 = OFF, 1..255 = số giây) */
int32_t app_mesh_SendHealthAttentionSet(uint16_t u16DstAddr, uint8_t u8Seconds);

/** @brief Health Period Set (Fast Period Divisor: 0..15) */
int32_t app_mesh_SendHealthPeriodSet(uint16_t u16DstAddr, uint8_t u8Divisor);

/** @brief Health Fault Clear */
int32_t app_mesh_SendHealthFaultClear(uint16_t u16DstAddr, uint16_t u16CompanyId);

/** @brief Health Fault Test */
int32_t app_mesh_SendHealthFaultTest(uint16_t u16DstAddr, uint16_t u16CompanyId, uint8_t u8TestId);

/** @brief Hàm tương thích cũ cho Health Fault Get */
int32_t app_mesh_HealthFaultGet(uint16_t dst);

/* ================== Console / Debug ================== */

/**
 * @brief In trạng thái Mesh (Provisioned, Unicast, NetIdx, AppIdx...)
 */
void app_mesh_PrintStatus(void);

/**
 * @brief In chi tiết tất cả model: AppKey đã bind, Group đã subscribe.
 *        Dùng cho lệnh console "meshmodels".
 */
void app_mesh_PrintModels(void);
