// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_attr.h>
#include <esp_log.h>

#include "canbus_mcp2515_handle.h"

#define TAG "canbus-mcp2515-isr"

void IRAM_ATTR mcp2515_interrupt_handler(void* isr_context) {
    // Get the MCP2515 driver context
    canbus_mcp2515_t* handle = (canbus_mcp2515_t*) isr_context;

    // TODO: Clarify the correct way to acquire the semaphore to wake the task up
    BaseType_t semaphoreGive = xSemaphoreGiveFromISR(handle->isr_task_semaphore, NULL);
    if (semaphoreGive != pdTRUE) {
        // TODO: Clarify that the strings below get set in IRAM as needed
        // TODO: Verify we have the correct log function based on the IRAM configuration

#ifdef CONFIG_MCP2515_ISR_IN_IRAM
    ESP_DRAM_LOGE(TAG, "Failed to process MCP2515 interrupt");
#else
    ESP_LOGE(TAG, "Failed to process MCP2515 interrupt");
#endif
    }
}