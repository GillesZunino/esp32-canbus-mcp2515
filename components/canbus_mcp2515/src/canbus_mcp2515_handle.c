// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_check.h>

#include "canbus_mcp2515_logtag.h"
#include "canbus_mcp2515_handle.h"


esp_err_t validate_mcp2515_handle(canbus_mcp2515_handle_t handle) {
    if (handle == NULL) {
#if CONFIG_MCP2515_ENABLE_DEBUG_LOG
        ESP_LOGE(CanBusMCP2515LogTag, "'handle' must not be NULL");
#endif
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->spi_device_handle == NULL) {
#if CONFIG_MCP2515_ENABLE_DEBUG_LOG
        ESP_LOGE(CanBusMCP2515LogTag, "'handle' has not been initialized via canbus_mcp2515_init()");
#endif
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}