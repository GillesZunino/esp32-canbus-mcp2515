// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <string.h>

#include <esp_check.h>

#include "canbus_mcp2515.h"

static const char* CanBusMCP2515LogTag = "canbus-mcp2515";

typedef struct canbus_mcp2515 {
    mcp2515_bit_timing_config_t bit_timing_config;
    spi_device_handle_t spi_device_handle;
} canbus_mcp2515_t;


static esp_err_t internal_check_mcp2515_config(const mcp2515_config_t* config);
static esp_err_t internal_check_mcp2515_in_configuration_mode(const canbus_mcp2515_handle_t handle);


esp_err_t canbus_mcp2515_init(const mcp2515_config_t* config, canbus_mcp2515_handle_t* handle) {
    *handle = NULL;

    // Basic configuration validation - Some parts of the configuration will be validated later by the SPI driver
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_config(config), CanBusMCP2515LogTag, "%s() Invalid configuration", __func__);

    // Allocate space for our handle
    canbus_mcp2515_t* pMcp2515 = calloc(1, sizeof(canbus_mcp2515_t));
    if (pMcp2515 == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Add an SPI device on the given bus - We accept the SPI bus configuration as is
    spi_device_interface_config_t spiDeviceInterfaceConfig = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,

        // We prefer mode = 0 [(0,0)] because it avoids one known issue with the MCP2515 - See https://ww1.microchip.com/downloads/en/DeviceDoc/80000179H.pdf
        .mode = 0,

        .clock_source = config->spi_cfg.clock_source,
        .clock_speed_hz = config->spi_cfg.clock_speed_hz,
        .input_delay_ns = config->spi_cfg.input_delay_ns,

        .spics_io_num = config->spi_cfg.spics_io_num,

        .flags = 0,
        .queue_size = config->spi_cfg.queue_size
    };

    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(spi_bus_add_device(config->spi_cfg.host_id, &spiDeviceInterfaceConfig, &pMcp2515->spi_device_handle), cleanup, CanBusMCP2515LogTag, "%s() Failed to spi_bus_add_device()", __func__);
    
    *handle = pMcp2515;

    return ret;

cleanup:
    if (pMcp2515 != NULL) {
        free(pMcp2515);
    }

    return ret;
}

esp_err_t canbus_mcp2515_free(canbus_mcp2515_handle_t handle) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Track the first error we encounter so we can return it to the caller
    esp_err_t firstError = ESP_OK;

    // Reset the MCP2515 before shutting the driver down
    esp_err_t err = canbus_mcp2515_reset(handle);
    if (err != ESP_OK) {
        firstError = err;
        ESP_LOGW(CanBusMCP2515LogTag, "%s() Failed to reset MCP2515 (%d)", __func__, err);
    }

    // Remove the device from the bus
    err = spi_bus_remove_device(handle->spi_device_handle);
    if (err != ESP_OK) {
        firstError = firstError == ESP_OK ? err : firstError;
        ESP_LOGW(CanBusMCP2515LogTag, "%s() Failed to remove MCP2515 device from SPI bus (%d)", __func__, err);
    }

    // Release memory
    free(handle);
    
    return firstError;
}

esp_err_t canbus_mcp2515_reset(canbus_mcp2515_handle_t handle) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    //
    // Reset MCP2515 - This sets configuration mode automatically
    //   * MOSI | 0xC0 |
    //   * MISO | N/A  |
    //
    const uint8_t transactionLengthInBytes = 1;
    spi_transaction_t spiTransaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = transactionLengthInBytes * 8,
        .rxlength = 0,
        .tx_data = { MCP2515_INSTRUCTION_RESET }
    };

    return spi_device_transmit(handle->spi_device_handle, &spiTransaction);
}

esp_err_t canbus_mcp2515_get_mode(const canbus_mcp2515_handle_t handle, mcp2515_mode_t* pMode) {
    if (pMode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t canStat = 0;
    esp_err_t err = mcp2515_read_register(handle, MCP2515_CANSTAT, &canStat);
    if (err == ESP_OK) {
        *pMode = (mcp2515_mode_t) canStat >> 5;
        return ESP_OK;
    }

    return err;
}

esp_err_t canbus_mcp2515_set_mode(canbus_mcp2515_handle_t handle, const mcp2515_mode_t mode, const TickType_t modeChangeDelay) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Try to set the desired mode - The mode will not change until all send / receive have completed
    esp_err_t err = mcp2515_modify_register(handle, MCP2515_CANCTRL, mode << 5, 0xE0);
    if (err == ESP_OK) {
        vTaskDelay(modeChangeDelay);

        uint8_t canStat = 0;
        err = mcp2515_read_register(handle, MCP2515_CANSTAT, &canStat);
        if (err == ESP_OK) {
            uint8_t opMd = canStat >> 5;
            bool currentModeMatchesDesired = opMd == mode;
            if (!currentModeMatchesDesired) {
                ESP_LOGE(CanBusMCP2515LogTag, "%s() Failed to set MCP2515 mode to %d, current mode is %d", __func__, mode, opMd);
            }
            return currentModeMatchesDesired ? ESP_OK : ESP_FAIL;
        }
    }

    return err;
}

esp_err_t canbus_mcp2515_set_bitrate(canbus_mcp2515_handle_t handle, const mcp2515_bit_timing_config_t* bitTimingConfig) {
    if ((bitTimingConfig->bit_rate_prescaler < 1) || (bitTimingConfig->bit_rate_prescaler > 64)) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((bitTimingConfig->sjw < 1) || (bitTimingConfig->sjw > 4)) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((bitTimingConfig->propagation_seg < 1) || (bitTimingConfig->propagation_seg > 8)) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((bitTimingConfig->phase_seg1 < 1) || (bitTimingConfig->phase_seg1 > 8)) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((bitTimingConfig->phase_seg2 < 1) || (bitTimingConfig->phase_seg2 > 8)) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP2515 needs to be in configuration mode to change bit timing
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_in_configuration_mode(handle), CanBusMCP2515LogTag, "%s() MCP2515 is not in configuration mode", __func__);

    // Copy bit configuration so we can re-apply it as needed
    handle->bit_timing_config = *bitTimingConfig;

    // Calculate configuration register values
    uint8_t cnf1 = ((bitTimingConfig->sjw & 0x03) << 6) | (bitTimingConfig->bit_rate_prescaler & 0x3F);

    uint8_t cnf2 = ((bitTimingConfig->btl_mode & 0x01) << 7) | ((bitTimingConfig->sample_mode & 0x01) << 6);
    cnf2 |= ((bitTimingConfig->phase_seg1 & 0x07) << 3) | (bitTimingConfig->propagation_seg & 0x07);

    uint8_t cnf3 = 0;
    if (bitTimingConfig->btl_mode == MCP2515_BTL_MODE_EXPLICIT) {
        cnf3 |= (bitTimingConfig->phase_seg2 & 0x07);
    }

    ESP_LOGI(CanBusMCP2515LogTag, "Configuring MCP2515 bit timing with CNF1: 0x%02X, CNF2: 0x%02X, CNF3[5:0]: 0x%02X", cnf1, cnf2, cnf3);
    
    // Apply bit timing configuration to chip - First CNF3 modify
    esp_err_t err = mcp2515_modify_register(handle, MCP2515_CNF3, cnf3, 0x07);
    if (err == ESP_OK) {
        // Then CNF2 and CNF1 with register address auto increment
        uint8_t cnf2_cnf1[] = { cnf2, cnf1 };
        err = mcp2515_write_registers(handle, MCP2515_CNF2, cnf2_cnf1, sizeof(cnf2_cnf1) / sizeof(cnf2_cnf1[0]));
    }

    return err;
}

esp_err_t canbus_mcp2515_set_receive_filter(canbus_mcp2515_handle_t handle, const mcp2515_receive_filter_t* filter) {
    if (filter == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP2515 needs to be in configuration mode to set filtering
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_in_configuration_mode(handle), CanBusMCP2515LogTag, "%s() MCP2515 is not in configuration mode", __func__);

    // Detect which filter mask register to use
    mcp2515_RXMn_t filterRegister;
    switch (filter->receive_register) {
        case RXF0:
        case RXF1:
            filterRegister = RXM0;
            break;

        case RXF2:
        case RXF3:
        case RXF4:
        case RXF5:
            filterRegister = RXM1;
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    // Use the correct register based on the mode
    switch (filter->mode) {
        case MCP2515_FILTER_STANDARD_FRAME:

            break;
        case MCP2515_FILTER_EXTENDED_FRAME:
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}
esp_err_t canbus_mcp2515_get_transmit_error_count(canbus_mcp2515_handle_t handle, uint8_t* count) {
    return mcp2515_read_register(handle, MCP2515_TEC, count);
}

esp_err_t canbus_mcp2515_get_receive_error_count(canbus_mcp2515_handle_t handle, uint8_t* count) {
    return mcp2515_read_register(handle, MCP2515_REC, count);
}

esp_err_t mcp2515_read_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, uint8_t* data) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

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

esp_err_t mcp2515_write_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data) {
    // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

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
    if (count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }


    // Prepare the command buffer with the povided payload
    const size_t transactionLenInBytes = 2UL + count;
    uint8_t writeBuffer[transactionLenInBytes];
    writeBuffer[0] = MCP2515_INSTRUCTION_WRITE;
    writeBuffer[1] = mcp2515RegisterStart;
    memcpy(&writeBuffer[2], data, count);

    //
    // Write Register Instruction format:
    //   * MOSI | 0x02 | <Register Address> | <Data> | <Data> | ... | <Data> |
    //   * MISO | N/A  | N/A                | N/A    | N/A    | ... | N/A    |
    //
    spi_transaction_t spiTransaction = {
        .flags = 0,
        .length = transactionLenInBytes * 8,
        .rxlength = 0,
        .tx_buffer = writeBuffer
    };

    return spi_device_transmit(handle->spi_device_handle, &spiTransaction);
}

esp_err_t mcp2515_modify_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data, const uint8_t mask) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

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
            return ESP_ERR_INVALID_ARG;
    }

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

static esp_err_t internal_check_mcp2515_config(const mcp2515_config_t* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    return ESP_OK;
}

static esp_err_t internal_check_mcp2515_in_configuration_mode(const canbus_mcp2515_handle_t handle) {
    mcp2515_mode_t mode;
    esp_err_t err = canbus_mcp2515_get_mode(handle, &mode);
    if (err == ESP_OK) {
        return mode == MCP2515_MODE_CONFIG ? ESP_OK : ESP_ERR_NOT_ALLOWED;
    }

    return err;
}