#pragma once 

#include "app_common.h"

/**
 * @brief Khởi tạo BLE Mesh Node (Config Server + Provisioning)
*/
int32_t app_mesh_Init(void);

/**
 * @brief In trạng thái Mesh hiện tại
 */
void    app_mesh_PrintStatus(void);

/**
 * @brief Kiểm tra node đã được provision chưa
 */
bool    app_mesh_IsProvisioned(void); 

/**
 * @brief Reset node về trạng thái Unprovisioned
 */
int32_t app_mesh_Reset(void);

/**
 * @brief Gửi lệnh Generic OnOff
 */
int32_t app_mesh_SendOnOff(uint16_t u16DstAddr, uint8_t u8OnOff);

/**
 * @brief Lấy trạng thái OnOff hiện tại của node
 */
uint8_t app_mesh_GetOnOffState(void);