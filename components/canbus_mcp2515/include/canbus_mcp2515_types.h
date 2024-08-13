// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MCP2515 modes of operation.
 * @note On power on, the MCP2515 automatically enters configuration mode.
 */
typedef enum {
    MCP2515_MODE_NORMAL     = 0x00,
    MCP2515_MODE_SLEEP      = 0x01,
    MCP2515_MODE_LOOPBACK   = 0x02,
    MCP2515_MODE_LISTENONLY = 0x03,
    MCP2515_MODE_CONFIG     = 0x04
} mcp2515_mode_t;

/**
 * @brief MCP2515 instructions.
 */
typedef enum {
    MCP2515_INSTRUCTION_WRITE       = 0x02,
    MCP2515_INSTRUCTION_READ        = 0x03,
    MCP2515_INSTRUCTION_BITMOD      = 0x05,
    MCP2515_INSTRUCTION_RESET       = 0xC0
}  mcp2515_instruction_t;


/**
 * @brief MCP2515 registers.
 */
typedef enum {
    MCP2515_CANSTAT  = 0x0E,
    MCP2515_CANCTRL  = 0x0F,
    MCP2515_CNF3     = 0x28,
    MCP2515_CNF2     = 0x29,
    MCP2515_CNF1     = 0x2A,
} mcp2515_register_t;


#ifdef __cplusplus
}
#endif