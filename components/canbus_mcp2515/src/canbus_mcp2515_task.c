// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <freertos/FreeRTOS.h>

#include "canbus_mcp2515.h"
#include "canbus_mcp2515_handle.h"
#include "canbus_mcp2515_registers.h"


void canbus_mcp2515_task(void* arg) {
    canbus_mcp2515_t* handle = (canbus_mcp2515_t*) arg;
    bool shouldExit = false;

    do {
        // Wait for an interrupt or a request to shutdown

        //TODO: Acquire the semaphore to wait for an interrupt
        // xSemaphoreTake();

        // TODO: Create a function instead of calling SPI directly
        uint8_t canStat = 0;
        esp_err_t err = mcp2515_read_register(handle, MCP2515_CANSTAT, &canStat);
        if (err == ESP_OK) {
            mcp2515_event_t event = (canStat & 0x0E) >> 1;
            if (event != MCP2515_EVENT_NONE) {
                mcp2515_event_parameters_t params = {};

                switch (event) {
                    case MCP2515_EVENT_ERROR:
                        // TODO: Do we need to pass arguments
                        //params.event_error.eflg = 0;
                        break;
                    case MCP2515_EVENT_WAKEUP:
                        // TODO
                        break;
                    case MCP2515_EVENT_TX0_MSG_SENT:
                    case MCP2515_EVENT_TX1_MSG_SENT:
                    case MCP2515_EVENT_TX2_MSG_SENT:
                        // TODO
                        break;
                    case MCP2515_EVENT_RX0_MSG_RECEIVED:
                    case MCP2515_EVENT_RX1_MSG_RECEIVED:
                        // TODO
                        break;
                    default:
                        break;
                }

                // Make a local copy  of the event handler in case it is re-configured from another task
                mcp2515_events_config_t events_config_local = {0};
                portENTER_CRITICAL_SAFE(&handle->spinLock);
                    events_config_local = handle->events_config;
                portEXIT_CRITICAL_SAFE(&handle->spinLock);

                // Invoke user callback
                if (events_config_local.handler != NULL) {
                    events_config_local.handler(event, &params, events_config_local.context);
                }
            }
        } else {
            // TODO: Log and also offer a way to recover so we do not end up in a loop
        }


        // TODO: Return the semaphore as needed
        // xSemaphoreGive();

        // TODO: Find a way to signal to the task that it needs to exit

    } while (!shouldExit);

    // TODO: Task delete when it exits
}