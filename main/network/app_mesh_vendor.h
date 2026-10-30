#pragma once

#include <stdint.h>
#include "esp_ble_mesh_defs.h"

/* ================== Vendor Model ================== */
#define DF_MESH_VND_COMPANY_ID          0x8888
#define DF_MESH_VND_MODEL_ID_SERVER     0x0000

/* Helper: tạo 3-byte vendor opcode từ 1-byte opcode + Company ID */
#define DF_VND_OPCODE(n)                ESP_BLE_MESH_MODEL_OP_3((n), DF_MESH_VND_COMPANY_ID)
