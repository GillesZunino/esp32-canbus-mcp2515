// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_check.h>
#include <driver/spi_master.h> 

#include "canbus_mcp2515_logtag.h"
#include "canbus_mcp2515_instructions.h"


esp_err_t mcp2515_send_single_byte_instruction(canbus_mcp2515_handle_t handle, const mcp2515_instruction_t instruction) {
    ESP_RETURN_ON_ERROR(validate_mcp2515_handle(handle), CanBusMCP2515LogTag, "'handle' in invalid");

    // Validate the instruction is single byte out, no data in
    switch (instruction) {
        case MCP2515_INSTRUCTION_RESET:
        case MCP2515_INSTRUCTION_RTS_TX0:
        case MCP2515_INSTRUCTION_RTS_TX1:
        case MCP2515_INSTRUCTION_RTS_TX2:
            break;

        default:
#ifdef CONFIG_MCP2515_ENABLE_DEBUG_LOG
            ESP_LOGE(CanBusMCP2515LogTag, "'instruction' value %d [%s] is invalid or not a single byte instruction", instruction, dump_mcp2515_instruction(instruction));
#else
            ESP_LOGE(CanBusMCP2515LogTag, "'instruction' value %d is invalid or not a single byte instruction", instruction);
#endif
            return ESP_ERR_INVALID_ARG;
    }

    return unchecked_mcp2515_send_single_byte_instruction(handle, instruction);
}

esp_err_t unchecked_mcp2515_send_single_byte_instruction(canbus_mcp2515_handle_t handle, const mcp2515_instruction_t instruction) {
    //
    // One byte MCP2515 instruction
    //   * MOSI | <Instruction> |
    //   * MISO |      N/A      |
    //
    const uint8_t transactionLengthInBytes = 1;
    spi_transaction_t spiTransaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = transactionLengthInBytes * 8,
        .rxlength = 0,
        .tx_data = { instruction }
    };

    return spi_device_transmit(handle->spi_device_handle, &spiTransaction);
}

const char* dump_mcp2515_instruction(const mcp2515_instruction_t instruction) {
    switch (instruction) {
        case MCP2515_INSTRUCTION_WRITE:           return "WRITE";
        case MCP2515_INSTRUCTION_READ:            return "READ";
        case MCP2515_INSTRUCTION_BITMOD:          return "BIT MODIFY";

        case MCP2515_INSTRUCTION_LOAD_TXB0SIDH:   return "LOAD TX BUFFER B0SIDH";
        case MCP2515_INSTRUCTION_LOAD_TXB0D0:     return "LOAD TX BUFFER B0D0";
        case MCP2515_INSTRUCTION_LOAD_TXB1SIDH:   return "LOAD TX BUFFER B1SIDH";
        case MCP2515_INSTRUCTION_LOAD_TXB1D0:     return "LOAD TX BUFFER B1D0";
        case MCP2515_INSTRUCTION_LOAD_TXB2SIDH:   return "LOAD TX BUFFER B2SIDH";
        case MCP2515_INSTRUCTION_LOAD_TXB2D0:     return "LOAD TX BUFFER B2D0";

        case MCP2515_INSTRUCTION_RTS_TX0:         return "RTS TX0";
        case MCP2515_INSTRUCTION_RTS_TX1:         return "RTS TX1";
        case MCP2515_INSTRUCTION_RTS_TX2:         return "RTS TX2";
        case MCP2515_INSTRUCTION_RTS_ALL:         return "RTS ALL";

        case MCP2515_INSTRUCTION_READ_RXB0SIDH:   return "READ RX BUFFER RXB0SIDH";
        case MCP2515_INSTRUCTION_READ_RXB0D0:     return "READ RX BUFFER RXB0D0";
        case MCP2515_INSTRUCTION_READ_RXB1SIDH:   return "READ RX BUFFER RXB1SIDH";
        case MCP2515_INSTRUCTION_READ_RXB1D0:     return "READ RX BUFFER RXB1D0";

        case MCP2515_INSTRUCTION_READ_STATUS:     return "READ STATUS";
        case MCP2515_INSTRUCTION_RX_STATUS:       return "RX STATUS";
        case MCP2515_INSTRUCTION_RESET:           return "RESET";
        
        default:                                  return "<unknown>";
    }
}