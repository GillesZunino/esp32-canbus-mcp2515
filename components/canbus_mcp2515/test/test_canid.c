// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_log.h>

#include "../src/canid_mcp2515_coding_private.h"

#include "unity.h"


#define TAG "[canid]"


TEST_CASE("CAN Standard ID Encoding", TAG) {
    ESP_LOGI(TAG, "Standard CAN ID encoding test starting");

    // Scan all ids in the CAN V2.0B Standard space, encode for MCP2515, decode and verify match
    const bool isExtendedFrame = false;
    for (uint32_t canId = 0; canId <= 0x7FF; canId++) {
        uint8_t buffer[4] = {0};
        encode_canid_into_mcp2515_registers_private(canId, isExtendedFrame, buffer);
        uint32_t decodedCanId = decode_canid_from_mcp2515_registers_private(isExtendedFrame, buffer);

        if (canId != decodedCanId) {
            ESP_LOGE(TAG, "Decoded standard frame 0x%lX does not match original 0x%lX", decodedCanId, canId);
        }

        TEST_ASSERT_EQUAL_UINT32_MESSAGE(canId, decodedCanId, "Decoded Standard CAN ID must match original CAN ID");
    }

    ESP_LOGI(TAG, "Standard CAN ID encoding test complete");
}

TEST_CASE("CAN Extended ID Encoding", TAG) {
    ESP_LOGI(TAG, "Extended CAN ID encoding test starting");


    // Scan all ids in the CAN V2.0B Extended space, encode for MCP2515, decode and verify match
    const bool isExtendedFrame = true;
    for (uint32_t canId = 0; canId <=  0x1FFFFFFFUL; canId++) {
        uint8_t buffer[4] = {0};
        encode_canid_into_mcp2515_registers_private(canId, isExtendedFrame, buffer);
        uint32_t decodedCanId = decode_canid_from_mcp2515_registers_private(isExtendedFrame, buffer);

        if (canId != decodedCanId) {
            ESP_LOGE(TAG, "Decoded extended frame 0x%lX does not match original 0x%lX", decodedCanId, canId);
        }

        TEST_ASSERT_EQUAL_UINT32_MESSAGE(canId, decodedCanId, "Decoded Extended CAN ID must match original CAN ID");
    }

    ESP_LOGI(TAG, "Extended CAN ID encoding test complete");
}