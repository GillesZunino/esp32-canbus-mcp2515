// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include <esp_err.h>

#include "canbus_mcp2515_handle.h"


#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief MCP2515 instructions.
 */
typedef enum {
    MCP2515_INSTRUCTION_WRITE           = 0x02,
    MCP2515_INSTRUCTION_READ            = 0x03,
    MCP2515_INSTRUCTION_BITMOD          = 0x05,

    MCP2515_INSTRUCTION_LOAD_TXB0SIDH   = 0x40,
    MCP2515_INSTRUCTION_LOAD_TXB0D0     = 0x41,
    MCP2515_INSTRUCTION_LOAD_TXB1SIDH   = 0x42,
    MCP2515_INSTRUCTION_LOAD_TXB1D0     = 0x43,
    MCP2515_INSTRUCTION_LOAD_TXB2SIDH   = 0x44,
    MCP2515_INSTRUCTION_LOAD_TXB2D0     = 0x45,

    MCP2515_INSTRUCTION_RTS_TX0         = 0x81,
    MCP2515_INSTRUCTION_RTS_TX1         = 0x82,
    MCP2515_INSTRUCTION_RTS_TX2         = 0x84,
    MCP2515_INSTRUCTION_RTS_ALL         = MCP2515_INSTRUCTION_RTS_TX0 | MCP2515_INSTRUCTION_RTS_TX1 | MCP2515_INSTRUCTION_RTS_TX2,

    MCP2515_INSTRUCTION_READ_RXB0SIDH   = 0x90,
    MCP2515_INSTRUCTION_READ_RXB0D0     = 0x92,
    MCP2515_INSTRUCTION_READ_RXB1SIDH   = 0x94,
    MCP2515_INSTRUCTION_READ_RXB1D0     = 0x96,

    MCP2515_INSTRUCTION_READ_STATUS     = 0xA0,
    MCP2515_INSTRUCTION_RX_STATUS       = 0xB0,
    MCP2515_INSTRUCTION_RESET           = 0xC0
}  mcp2515_instruction_t;


/**
 * @brief Send a one byte command to the MCP2515.
 * @param handle       Handle of the MCP2515 device
 * @param instruction  Instruction to send
 */
esp_err_t mcp2515_send_single_byte_instruction(canbus_mcp2515_handle_t handle, const mcp2515_instruction_t instruction);

/**
 * @brief Send a one byte command to the MCP2515.
 * @param handle       Handle of the MCP2515 device
 * @param instruction  Instruction to send
 * @attention This function does not validate arguments and should only be called from within the driver.
 */
esp_err_t unchecked_mcp2515_send_single_byte_instruction(canbus_mcp2515_handle_t handle, const mcp2515_instruction_t instruction);


/**
 * @brief Dump a MCP2515 instruction to a string.
 * @param instruction  Instruction to dump
 * @return A string representation of the instruction.
 */
const char* dump_mcp2515_instruction(const mcp2515_instruction_t instruction);



#ifdef __cplusplus
}
#endif