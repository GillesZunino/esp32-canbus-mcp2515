// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_log.h>

#include "canbus_mcp2515_utils.h"

#include "unity.h"


#define TAG "[registerlog]"


TEST_CASE("Dump CANINTE", TAG) {
    uint8_t caninte = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_CANINTE(TAG, caninte, ESP_LOG_INFO);
}

TEST_CASE("Dump CANINTF", TAG) {
    uint8_t canintf = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_CANINTF(TAG, canintf, ESP_LOG_INFO);
}

TEST_CASE("Dump EFLG", TAG) {
    uint8_t eflg = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_EFLG(TAG, eflg, ESP_LOG_INFO);
}

TEST_CASE("Dump CANSTAT", TAG) {
    uint8_t canstat = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1);
    MCP2515_LOG_CANSTAT(TAG, canstat, ESP_LOG_INFO);
}

TEST_CASE("Dump CANCTRL", TAG) {
    uint8_t canctrl = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_CANCTRL(TAG, canctrl, ESP_LOG_INFO);
}

TEST_CASE("Dump TXBnCTRL", TAG) {
    uint8_t txbnctrl = (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_TXBnCTRL(TAG, txbnctrl, ESP_LOG_INFO);
}

TEST_CASE("Dump RXB0CTRL", TAG) {
    uint8_t rxb0ctrl = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_RXB0CTRL(TAG, rxb0ctrl, ESP_LOG_INFO);
}

TEST_CASE("Dump RXB1CTRL", TAG) {
    uint8_t rxb1ctrl = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0);
    MCP2515_LOG_RXB1CTRL(TAG, rxb1ctrl, ESP_LOG_INFO);
}