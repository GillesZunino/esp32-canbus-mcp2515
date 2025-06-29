// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include <esp_err.h>

#include "canbus_mcp2515_handle.h"


#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief MCP2515 registers.
 */
typedef enum {
    MCP2515_RXF0SIDH  = 0x00,
    MCP2515_RXF0SIDL  = 0x01,
    MCP2515_RXF0EID8  = 0x02,
    MCP2515_RXF0EID0  = 0x03,

    MCP2515_RXF1SIDH  = 0x04,
    MCP2515_RXF1SIDL  = 0x05,
    MCP2515_RXF1EID8  = 0x06,
    MCP2515_RXF1EID0  = 0x07,

    MCP2515_RXF2SIDH  = 0x08,
    MCP2515_RXF2SIDL  = 0x09,
    MCP2515_RXF2EID8  = 0x0A,
    MCP2515_RXF2EID0  = 0x0B,

    MCP2515_BFPCTRL   = 0x0C,
    MCP2515_TXRTSCTRL = 0x0D,

    MCP2515_CANSTAT   = 0x0E,
    MCP2515_CANCTRL   = 0x0F,

    MCP2515_RXF3SIDH  = 0x10,
    MCP2515_RXF3SIDL  = 0x11,
    MCP2515_RXF3EID8  = 0x12,
    MCP2515_RXF3EID0  = 0x13,

    MCP2515_RXF4SIDH  = 0x14,
    MCP2515_RXF4SIDL  = 0x15,
    MCP2515_RXF4EID8  = 0x16,
    MCP2515_RXF4EID0  = 0x17,

    MCP2515_RXF5SIDH  = 0x18,
    MCP2515_RXF5SIDL  = 0x19,
    MCP2515_RXF5EID8  = 0x1A,
    MCP2515_RXF5EID0  = 0x1B,

    MCP2515_TEC       = 0x1C,
    MCP2515_REC       = 0x1D,

    MCP2515_RXM0SIDH  = 0x20,
    MCP2515_RXM0SIDL  = 0x21,
    MCP2515_RXM0EID8  = 0x22,
    MCP2515_RXM0EID0  = 0x23,

    MCP2515_RXM1SIDH  = 0x24,
    MCP2515_RXM1SIDL  = 0x25,
    MCP2515_RXM1EID8  = 0x26,
    MCP2515_RXM1EID0  = 0x27,

    MCP2515_CNF3      = 0x28,
    MCP2515_CNF2      = 0x29,
    MCP2515_CNF1      = 0x2A,

    MCP2515_CANINTE   = 0x2B,
    MCP2515_CANINTF   = 0x2C,

    MCP2515_EFLG      = 0x2D,

    MCP2515_TXB0CTRL  = 0x30,
    MCP2515_TXB0SIDH  = 0x31,
    MCP2515_TXB0SIDL  = 0x32,
    MCP2515_TXB0EID8  = 0x33,
    MCP2515_TXB0EID0  = 0x34,

    MCP2515_TXB0DLC   = 0x35,

    MCP2515_TXB0D0    = 0x36,
    MCP2515_TXB0D1    = 0x37,
    MCP2515_TXB0D2    = 0x38,
    MCP2515_TXB0D3    = 0x39,
    MCP2515_TXB0D4    = 0x3A,
    MCP2515_TXB0D5    = 0x3B,
    MCP2515_TXB0D6    = 0x3C,
    MCP2515_TXB0D7    = 0x3D,

    MCP2515_TXB1CTRL  = 0x40,
    MCP2515_TXB1SIDH  = 0x41,
    MCP2515_TXB1SIDL  = 0x42,
    MCP2515_TXB1EID8  = 0x43,
    MCP2515_TXB1EID0  = 0x44,

    MCP2515_TXB1DLC   = 0x45,

    MCP2515_TXB1D0    = 0x46,
    MCP2515_TXB1D1    = 0x47,
    MCP2515_TXB1D2    = 0x48,
    MCP2515_TXB1D3    = 0x49,
    MCP2515_TXB1D4    = 0x4A,
    MCP2515_TXB1D5    = 0x4B,
    MCP2515_TXB1D6    = 0x4C,
    MCP2515_TXB1D7    = 0x4D,

    MCP2515_TXB2CTRL  = 0x50,
    MCP2515_TXB2SIDH  = 0x51,
    MCP2515_TXB2SIDL  = 0x52,
    MCP2515_TXB2EID8  = 0x53,
    MCP2515_TXB2EID0  = 0x54,

    MCP2515_TXB2DLC   = 0x55,

    MCP2515_TXB2D0    = 0x56,
    MCP2515_TXB2D1    = 0x57,
    MCP2515_TXB2D2    = 0x58,
    MCP2515_TXB2D3    = 0x59,
    MCP2515_TXB2D4    = 0x5A,
    MCP2515_TXB2D5    = 0x5B,
    MCP2515_TXB2D6    = 0x5C,
    MCP2515_TXB2D7    = 0x5D,

    MCP2515_RXB0CTRL  = 0x60,
    MCP2515_RXB0SIDH  = 0x61,
    MCP2515_RXB0SIDL  = 0x62,
    MCP2515_RXB0EID8  = 0x63,
    MCP2515_RXB0EID0  = 0x64,

    MCP2515_RXB0DLC   = 0x65,

    MCP2515_RXB0D0    = 0x66,
    MCP2515_RXB0D1    = 0x67,
    MCP2515_RXB0D2    = 0x68,
    MCP2515_RXB0D3    = 0x69,
    MCP2515_RXB0D4    = 0x6A,
    MCP2515_RXB0D5    = 0x6B,
    MCP2515_RXB0D6    = 0x6C,
    MCP2515_RXB0D7    = 0x6D,

    MCP2515_RXB1CTRL  = 0x70,
    MCP2515_RXB1SIDH  = 0x71,
    MCP2515_RXB1SIDL  = 0x72,
    MCP2515_RXB1EID8  = 0x73,
    MCP2515_RXB1EID0  = 0x74,

    MCP2515_RXB1DLC   = 0x75,

    MCP2515_RXB1D0    = 0x76,
    MCP2515_RXB1D1    = 0x77,
    MCP2515_RXB1D2    = 0x78,
    MCP2515_RXB1D3    = 0x79,
    MCP2515_RXB1D4    = 0x7A,
    MCP2515_RXB1D5    = 0x7B,
    MCP2515_RXB1D6    = 0x7C,
    MCP2515_RXB1D7    = 0x7D
} mcp2515_register_t;


/**
 * @brief Read an MCP2515 register.
 * @param handle          Handle of the MCP2515 device
 * @param mcp2515Register Register to read
 * @param data            Pointer to a memory location which receives the data
 */
esp_err_t mcp2515_read_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, uint8_t* data);

/**
 * @brief Read an MCP2515 register.
 * @param handle          Handle of the MCP2515 device
 * @param mcp2515Register Register to read
 * @param data            Pointer to a memory location which receives the data
 * @attention This function does not validate arguments and should only be called from within the driver.
 */
esp_err_t unchecked_mcp2515_read_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, uint8_t* data);

/**
 * @brief Read multiple MCP2515 registers in sequence.
 * @param handle          Handle of the MCP2515 device
 * @param mcp2515Register Register to start reading
 * @param data            Pointer to a memory location which receives the data
 */
esp_err_t mcp2515_read_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, uint8_t* data, const uint8_t count);

/**
 * @brief Read multiple MCP2515 registers in sequence.
 * @param handle          Handle of the MCP2515 device
 * @param mcp2515Register Register to start reading
 * @param data            Pointer to a memory location which receives the data
 * @attention This function does not validate arguments and should only be called from within the driver.
 */
esp_err_t unchecked_mcp2515_read_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, uint8_t* data, const uint8_t count);

/**
 * @brief Write to an MCP2515 register.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to write
 * @param data                  Data to write
 */
esp_err_t mcp2515_write_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data);

/**
 * @brief Write to an MCP2515 register.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to write
 * @param data                  Data to write
 * @attention This function does not validate arguments and should only be called from within the driver.
 */
esp_err_t unchecked_mcp2515_write_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data);

/**
 * @brief Write to multiple MCP2515 registers in sequence.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to start writing
 * @param data                  Data to write
 * @param count                 Number of bytes to write
 */
esp_err_t mcp2515_write_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, const uint8_t data[], const uint8_t count);

/**
 * @brief Write to multiple MCP2515 registers in sequence.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to start writing
 * @param data                  Data to write
 * @param count                 Number of bytes to write
 * @attention This function does not validate arguments and should only be called from within the driver.
 */
esp_err_t unchecked_mcp2515_write_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, const uint8_t data[], const uint8_t count);

/**
 * @brief Change a set of bits in a MCP2515 register.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to alter
 * @param data                  Data to apply to the register
 * @param mask                  Number of bytes to read
 */
esp_err_t mcp2515_modify_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data, const uint8_t mask);


/**
 * @brief Change a set of bits in a MCP2515 register.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to alter
 * @param data                  Data to apply to the register
 * @param mask                  Number of bytes to read
 * @attention This function does not validate arguments and should only be called from within the driver.
 */
esp_err_t unchecked_mcp2515_modify_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data, const uint8_t mask);


/**
 * @brief Validate an MCP2515 register value against the official list of MCP2515 registers.
 * @param mcp2515Register Register to validate
 * @return ESP_OK if the register is valid, or an error code if invalid.
 */
esp_err_t validate_mcp2515_register(const mcp2515_register_t mcp2515Register);

/**
 * @brief Dump a MCP2515 register to a string.
 * @param instruction  Register to dump
 * @return A string representation of the register.
**/
const char* dump_mcp2515_register(const mcp2515_register_t mcp2515Register);



#ifdef __cplusplus
}
#endif