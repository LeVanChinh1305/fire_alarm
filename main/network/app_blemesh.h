#pragma once 

#include "app_common.h"
#include "app_mesh_vendor.h"

/**
 * @brief Khởi tạo BLE Mesh Node (15 model: Config/Health/Generic/Lighting/Vendor)
*/
int32_t app_mesh_Init(void);

/**
 * @brief In trạng thái Mesh hiện tại
 */
void    app_mesh_PrintStatus(void);

/**
 * @brief In AppKey đã bind và group đã subscribe của từng model
 */
void    app_mesh_PrintModels(void);

/**
 * @brief Kiểm tra node đã được provision chưa
 */
bool    app_mesh_IsProvisioned(void); 

/**
 * @brief Reset node về trạng thái Unprovisioned
 */
int32_t app_mesh_Reset(void);

/**
 * @brief Gateway Join: tự thêm AppKey HOMI, bind model, subscribe Group 0xC000
 */
int32_t app_mesh_GatewayJoin(void);

/**
 * @brief Lấy trạng thái OnOff hiện tại của node
 */
uint8_t app_mesh_GetOnOffState(void);
