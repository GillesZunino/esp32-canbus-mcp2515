// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include "canbus_mcp2515.h"


#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief Internal structure to represent our connection to an MCP2515 device.
 * @attention This structure is internal and must not be accessed directly.
 */
typedef struct canbus_mcp2515 {
    portMUX_TYPE spinLock;
    SemaphoreHandle_t isr_task_semaphore;
    mcp2515_bit_timing_config_t bit_timing_config;
    mcp2515_rxnbf_pins_config_t rxnbf_config;
    mcp2515_events_config_t events_config;
    spi_device_handle_t spi_device_handle;
} canbus_mcp2515_t;


/**
 * @brief Validate the MCP2515 handle.
 * @param handle Handle to validate
 * @return ESP_OK if the handle is valid, or an error code if invalid.
 */
esp_err_t validate_mcp2515_handle(canbus_mcp2515_handle_t handle);



#ifdef __cplusplus
}
#endif