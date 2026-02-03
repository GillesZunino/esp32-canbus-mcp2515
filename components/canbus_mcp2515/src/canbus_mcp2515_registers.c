// -----------------------------------------------------------------------------------
// Copyright 2025, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <string.h>

#include <esp_check.h>

#include "canbus_mcp2515_logtag.h"
#include "canbus_mcp2515_handle.h"
#include "canbus_mcp2515_instructions.h"
#include "canbus_mcp2515_registers.h"


static esp_err_t internal_validate_mcp2515_multi_registers_datacount(const mcp2515_register_t mcp2515RegisterStart, const uint8_t count);


esp_err_t mcp2515_read_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, uint8_t* data) {
    ESP_RETURN_ON_ERROR(validate_mcp2515_handle(handle), CanBusMCP2515LogTag, "'handle' is invalid");
    ESP_RETURN_ON_ERROR(validate_mcp2515_register(mcp2515Register), CanBusMCP2515LogTag, "'mcp2515Register' must be a value from mcp2515_register_t");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, CanBusMCP2515LogTag, "'data' must not be NULL'");

    return unchecked_mcp2515_read_register(handle, mcp2515Register, data);
}

esp_err_t unchecked_mcp2515_read_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, uint8_t* data) {
    //
    // Read Register Instruction format:
    //   * MOSI | 0xC0 | <Register Address> |      |
    //   * MISO | N/A  | N/A                | Data |         
    //
    const uint8_t commandLengthInBytes = 2;
    const uint8_t responseLengthInBytes = 1;
    const uint8_t transactionLengthInBytes = commandLengthInBytes + responseLengthInBytes;
    spi_transaction_t spiTransaction = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = transactionLengthInBytes * 8,
        .rxlength = transactionLengthInBytes * 8,
        .tx_data = { MCP2515_INSTRUCTION_READ, mcp2515Register }
    };

    esp_err_t err = spi_device_transmit(handle->spi_device_handle, &spiTransaction);
    if (err == ESP_OK) {
        *data = spiTransaction.rx_data[transactionLengthInBytes - 1];
    }

    return err;
}

esp_err_t mcp2515_read_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, uint8_t* data, const uint8_t count) {
    ESP_RETURN_ON_ERROR(validate_mcp2515_handle(handle), CanBusMCP2515LogTag, "'handle' is invalid");
    ESP_RETURN_ON_ERROR(validate_mcp2515_register(mcp2515RegisterStart), CanBusMCP2515LogTag, "'mcp2515RegisterStart' must be a value from mcp2515_register_t");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, CanBusMCP2515LogTag, "'data' must not be NULL'");
    ESP_RETURN_ON_ERROR(internal_validate_mcp2515_multi_registers_datacount(mcp2515RegisterStart, count), CanBusMCP2515LogTag, "'count' is invalid");

    return unchecked_mcp2515_read_registers(handle, mcp2515RegisterStart, data, count);
}

esp_err_t unchecked_mcp2515_read_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, uint8_t* data, const uint8_t count) {
    //
    // Read Registers Instruction format:
    //   * MOSI | 0xC0 | <Register Address> |
    //   * MISO | N/A  | N/A                | <Data> | <Data> | ... | <Data> |
    //

    // Prepare the command buffer with the provided payload
    const uint8_t commandLengthInBytes = 2;
    const size_t transactionLengthInBytes = commandLengthInBytes + count;

    WORD_ALIGNED_ATTR uint8_t commandBuffer[commandLengthInBytes];
    commandBuffer[0] = MCP2515_INSTRUCTION_READ;
    commandBuffer[1] = mcp2515RegisterStart;

    uint8_t receiveBuffer[transactionLengthInBytes];

    spi_transaction_t spiTransaction = {
        .flags = 0,
        .length = transactionLengthInBytes * 8,
        .rxlength = transactionLengthInBytes * 8,
        .tx_buffer = commandBuffer,
        .rx_buffer = receiveBuffer
    };

    esp_err_t err = spi_device_transmit(handle->spi_device_handle, &spiTransaction);
    if (err == ESP_OK) {
       memcpy(data, &receiveBuffer[commandLengthInBytes], count);
    }

    return err;
}

esp_err_t mcp2515_write_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data) {
    ESP_RETURN_ON_ERROR(validate_mcp2515_handle(handle), CanBusMCP2515LogTag, "'handle' is invalid");
    ESP_RETURN_ON_ERROR(validate_mcp2515_register(mcp2515Register), CanBusMCP2515LogTag, "'mcp2515Register' must be a value from mcp2515_register_t");

    return unchecked_mcp2515_write_register(handle, mcp2515Register, data);
}

esp_err_t unchecked_mcp2515_write_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data) {
    //
    // Write Register Instruction format:
    //   * MOSI | 0x02 | <Register Address> |
    //   * MISO | N/A  | N/A                |
    //
    const uint8_t transactionLengthInBytes = 3;
    spi_transaction_t spiTransaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = transactionLengthInBytes * 8,
        .rxlength = 0,
        .tx_data = { MCP2515_INSTRUCTION_WRITE, mcp2515Register, data }
    };

    return spi_device_transmit(handle->spi_device_handle, &spiTransaction);
}

esp_err_t mcp2515_write_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, const uint8_t data[], const uint8_t count) {
    ESP_RETURN_ON_ERROR(validate_mcp2515_handle(handle), CanBusMCP2515LogTag, "'handle' is invalid");
    ESP_RETURN_ON_ERROR(validate_mcp2515_register(mcp2515RegisterStart), CanBusMCP2515LogTag, "'mcp2515RegisterStart' must be a value from mcp2515_register_t");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, CanBusMCP2515LogTag, "'data' must not be NULL'");
    ESP_RETURN_ON_ERROR(internal_validate_mcp2515_multi_registers_datacount(mcp2515RegisterStart, count), CanBusMCP2515LogTag, "'count' is invalid");

    return unchecked_mcp2515_write_registers(handle, mcp2515RegisterStart, data, count);
}

esp_err_t unchecked_mcp2515_write_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, const uint8_t data[], const uint8_t count) {
    //
    // Write Register Instruction format:
    //   * MOSI | 0x02 | <Register Address> | <Data> | <Data> | ... | <Data> |
    //   * MISO | N/A  | N/A                | N/A    | N/A    | ... | N/A    |
    //

    // Prepare the command buffer with the provided payload
    const size_t transactionLenInBytes = 2UL + count;
    WORD_ALIGNED_ATTR uint8_t writeBuffer[transactionLenInBytes];
    writeBuffer[0] = MCP2515_INSTRUCTION_WRITE;
    writeBuffer[1] = mcp2515RegisterStart;
    memcpy(&writeBuffer[2], data, count);

    spi_transaction_t spiTransaction = {
        .flags = 0,
        .length = transactionLenInBytes * 8,
        .rxlength = 0,
        .tx_buffer = writeBuffer
    };

    return spi_device_transmit(handle->spi_device_handle, &spiTransaction);
}

esp_err_t mcp2515_modify_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data, const uint8_t mask) {
    ESP_RETURN_ON_ERROR(validate_mcp2515_handle(handle), CanBusMCP2515LogTag, "'handle' is invalid");

    // Ensure the register can be modified via BITMOD
    switch (mcp2515Register) {
        case MCP2515_BFPCTRL:
        case MCP2515_TXRTSCTRL:
        case MCP2515_CANCTRL:
        case MCP2515_CNF3:
        case MCP2515_CNF2:
        case MCP2515_CNF1:
        case MCP2515_CANINTE:
        case MCP2515_CANINTF:
        case MCP2515_EFLG:
        case MCP2515_TXB0CTRL:
        case MCP2515_TXB1CTRL:
        case MCP2515_TXB2CTRL:
        case MCP2515_RXB0CTRL:
        case MCP2515_RXB1CTRL:
            break;
        default:
        {
#ifdef CONFIG_MCP2515_ENABLE_DEBUG_LOG
            ESP_LOGE(CanBusMCP2515LogTag, "'mcp2515Register' is unknown or invalid. Register '%s' (%d) cannot be altered via BITMOD instruction", dump_mcp2515_register(mcp2515Register), mcp2515Register);
#else
            ESP_LOGE(CanBusMCP2515LogTag, "'mcp2515Register' is unknown or invalid. Register %d cannot be altered via BITMOD instruction", mcp2515Register);
#endif
            return ESP_ERR_INVALID_ARG;
        }   
    }

    return unchecked_mcp2515_modify_register(handle, mcp2515Register, data, mask);
}

esp_err_t unchecked_mcp2515_modify_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data, const uint8_t mask) {
    //
    // Bit Modify Instruction format:
    //   * MOSI | 0x05 | <Register Address> | <Mask> | <Data> |
    //   * MISO |      |                    |        |        |
    //
    const uint8_t transactionLengthInBytes = 4;
    spi_transaction_t spiTransaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = transactionLengthInBytes * 8,
        .rxlength = 0,
        .tx_data = { MCP2515_INSTRUCTION_BITMOD, mcp2515Register, mask, data }
    };

    return spi_device_transmit(handle->spi_device_handle, &spiTransaction);
}

esp_err_t validate_mcp2515_register(const mcp2515_register_t mcp2515Register) {
    bool validRegister = (mcp2515Register >= MCP2515_RXF0SIDH) && (mcp2515Register <= MCP2515_RXB1D7);

#ifdef CONFIG_MCP2515_ENABLE_DEBUG_LOG
    if (!validRegister) {
        ESP_LOGE(CanBusMCP2515LogTag, "'mcp2515Register' %d is invalid", mcp2515Register);
    }
#endif

    return  validRegister ? ESP_OK : ESP_ERR_INVALID_ARG;
}

const char* dump_mcp2515_register(const mcp2515_register_t mcp2515Register) {
    switch (mcp2515Register) {
        case MCP2515_RXF0SIDH:  return "RXF0SIDH";
        case MCP2515_RXF0SIDL:  return "RXF0SIDL";
        case MCP2515_RXF0EID8:  return "RXF0EID8";
        case MCP2515_RXF0EID0:  return "RXF0EID0";

        case MCP2515_RXF1SIDH:  return "RXF1SIDH";
        case MCP2515_RXF1SIDL:  return "RXF1SIDL";
        case MCP2515_RXF1EID8:  return "RXF1EID8";
        case MCP2515_RXF1EID0:  return "RXF1EID0";

        case MCP2515_RXF2SIDH:  return "RXF2SIDH";
        case MCP2515_RXF2SIDL:  return "RXF2SIDL";
        case MCP2515_RXF2EID8:  return "RXF2EID8";
        case MCP2515_RXF2EID0:  return "RXF2EID0";

        case MCP2515_BFPCTRL:   return "BFPCTRL";
        case MCP2515_TXRTSCTRL: return "TXRTSCTRL";

        case MCP2515_CANSTAT:   return "CANSTAT";
        case MCP2515_CANCTRL:   return "CANCTRL";

        case MCP2515_RXF3SIDH:  return "RXF3SIDH";
        case MCP2515_RXF3SIDL:  return "RXF3SIDL";
        case MCP2515_RXF3EID8:  return "RXF3EID8";
        case MCP2515_RXF3EID0:  return "RXF3EID0";

        case MCP2515_RXF4SIDH:  return "RXF4SIDH";
        case MCP2515_RXF4SIDL:  return "RXF4SIDL";
        case MCP2515_RXF4EID8:  return "RXF4EID8";
        case MCP2515_RXF4EID0:  return "RXF4EID0";

        case MCP2515_RXF5SIDH:  return "RXF5SIDH";
        case MCP2515_RXF5SIDL:  return "RXF5SIDL";
        case MCP2515_RXF5EID8:  return "RXF5EID8";
        case MCP2515_RXF5EID0:  return "RXF5EID0";

        case MCP2515_TEC:       return "TEC";
        case MCP2515_REC:       return "REC";

        case MCP2515_RXM0SIDH:  return "RXM0SIDH";
        case MCP2515_RXM0SIDL:  return "RXM0SIDL";
        case MCP2515_RXM0EID8:  return "RXM0EID8";
        case MCP2515_RXM0EID0:  return "RXM0EID0";

        case MCP2515_RXM1SIDH:  return "RXM1SIDH";
        case MCP2515_RXM1SIDL:  return "RXM1SIDL";
        case MCP2515_RXM1EID8:  return "RXM1EID8";
        case MCP2515_RXM1EID0:  return "RXM1EID0";

        case MCP2515_CNF3:      return "CNF3";
        case MCP2515_CNF2:      return "CNF2";
        case MCP2515_CNF1:      return "CNF1";

        case MCP2515_CANINTE:   return "CANINTE";
        case MCP2515_CANINTF:   return "CANINTF";

        case MCP2515_EFLG:      return "EFLG";

        case MCP2515_TXB0CTRL:  return "TXB0CTRL";
        case MCP2515_TXB0SIDH:  return "TXB0SIDH";
        case MCP2515_TXB0SIDL:  return "TXB0SIDL";
        case MCP2515_TXB0EID8:  return "TXB0EID8";
        case MCP2515_TXB0EID0:  return "TXB0EID0";

        case MCP2515_TXB0DLC:   return "TXB0DLC";

        case MCP2515_TXB0D0:    return "TXB0D0";
        case MCP2515_TXB0D1:    return "TXB0D1";
        case MCP2515_TXB0D2:    return "TXB0D2";
        case MCP2515_TXB0D3:    return "TXB0D3";
        case MCP2515_TXB0D4:    return "TXB0D4";
        case MCP2515_TXB0D5:    return "TXB0D5";
        case MCP2515_TXB0D6:    return "TXB0D6";
        case MCP2515_TXB0D7:    return "TXB0D7";

        case MCP2515_TXB1CTRL:  return "TXB1CTRL";
        case MCP2515_TXB1SIDH:  return "TXB1SIDH";
        case MCP2515_TXB1SIDL:  return "TXB1SIDL";
        case MCP2515_TXB1EID8:  return "TXB1EID8";
        case MCP2515_TXB1EID0:  return "TXB1EID0";

        case MCP2515_TXB1DLC:   return "TXB1DLC";

        case MCP2515_TXB1D0:    return "TXB1D0";
        case MCP2515_TXB1D1:    return "TXB1D1";
        case MCP2515_TXB1D2:    return "TXB1D2";
        case MCP2515_TXB1D3:    return "TXB1D3";
        case MCP2515_TXB1D4:    return "TXB1D4";
        case MCP2515_TXB1D5:    return "TXB1D5";
        case MCP2515_TXB1D6:    return "TXB1D6";
        case MCP2515_TXB1D7:    return "TXB1D7";

        case MCP2515_TXB2CTRL:  return "TXB2CTRL";
        case MCP2515_TXB2SIDH:  return "TXB2SIDH";
        case MCP2515_TXB2SIDL:  return "TXB2SIDL";
        case MCP2515_TXB2EID8:  return "TXB2EID8";
        case MCP2515_TXB2EID0:  return "TXB2EID0";

        case MCP2515_TXB2DLC:   return "TXB2DLC";

        case MCP2515_TXB2D0:    return "TXB2D0";
        case MCP2515_TXB2D1:    return "TXB2D1";
        case MCP2515_TXB2D2:    return "TXB2D2";
        case MCP2515_TXB2D3:    return "TXB2D3";
        case MCP2515_TXB2D4:    return "TXB2D4";
        case MCP2515_TXB2D5:    return "TXB2D5";
        case MCP2515_TXB2D6:    return "TXB2D6";
        case MCP2515_TXB2D7:    return "TXB2D7";

        case MCP2515_RXB0CTRL:  return "RXB0CTRL";
        case MCP2515_RXB0SIDH:  return "RXB0SIDH";
        case MCP2515_RXB0SIDL:  return "RXB0SIDL";
        case MCP2515_RXB0EID8:  return "RXB0EID8";
        case MCP2515_RXB0EID0:  return "RXB0EID0";

        case MCP2515_RXB0DLC:   return "RXB0DLC";

        case MCP2515_RXB0D0:    return "RXB0D0";
        case MCP2515_RXB0D1:    return "RXB0D1";
        case MCP2515_RXB0D2:    return "RXB0D2";
        case MCP2515_RXB0D3:    return "RXB0D3";
        case MCP2515_RXB0D4:    return "RXB0D4";
        case MCP2515_RXB0D5:    return "RXB0D5";
        case MCP2515_RXB0D6:    return "RXB0D6";
        case MCP2515_RXB0D7:    return "RXB0D7";

        case MCP2515_RXB1CTRL:  return "RXB1CTRL";
        case MCP2515_RXB1SIDH:  return "RXB1SIDH";
        case MCP2515_RXB1SIDL:  return "RXB1SIDL";
        case MCP2515_RXB1EID8:  return "RXB1EID8";
        case MCP2515_RXB1EID0:  return "RXB1EID0";

        case MCP2515_RXB1DLC:   return "RXB1DLC";

        case MCP2515_RXB1D0:    return "RXB1D0";
        case MCP2515_RXB1D1:    return "RXB1D1";
        case MCP2515_RXB1D2:    return "RXB1D2";
        case MCP2515_RXB1D3:    return "RXB1D3";
        case MCP2515_RXB1D4:    return "RXB1D4";
        case MCP2515_RXB1D5:    return "RXB1D5";
        case MCP2515_RXB1D6:    return "RXB1D6";
        case MCP2515_RXB1D7:    return "RXB1D7";

        default:                return "<Unknown>";
    }
}

static esp_err_t internal_validate_mcp2515_multi_registers_datacount(const mcp2515_register_t mcp2515RegisterStart, const uint8_t count) {
    //
    // The MCP2515 automatically increments the register address when reading or writing, making it possible to read or write multiple contiguous registers in a single transaction.
    // The maximum number of bytes which can be read depend on the starting register - See Table 11-1 in the MCP2515 datasheet.
    //
    uint8_t columnIndex = mcp2515RegisterStart >> 4;
    uint8_t lineIndex = mcp2515RegisterStart & 0x0F;
    uint8_t registersCount = ((7 - columnIndex) * 16) + (16 - lineIndex);
    bool isCountValid = (count > 0) && (count <= registersCount);

#ifdef CONFIG_MCP2515_ENABLE_DEBUG_LOG
    if (!isCountValid) {
        ESP_LOGE(CanBusMCP2515LogTag, "'count' (%d) is invalid. When starting at [%s], 'count' must be > 0 and <= %d", count, dump_mcp2515_register(mcp2515RegisterStart), registersCount);
    }
#endif

    return isCountValid ? ESP_OK : ESP_ERR_INVALID_ARG;
}