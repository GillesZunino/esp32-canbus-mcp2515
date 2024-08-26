// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <string.h>

#include <esp_check.h>

#include "canbus_mcp2515.h"
#include "canbus_mcp2515_types.h"

static const char* CanBusMCP2515LogTag = "canbus-mcp2515";

typedef struct canbus_mcp2515 {
    mcp2515_bit_timing_config_t bit_timing_config;
    mcp2515_rxnbf_pins_config_t rxnbf_config;
    mcp2515_interrupt_config_t interrupt_config;
    spi_device_handle_t spi_device_handle;
} canbus_mcp2515_t;


static esp_err_t internal_check_mcp2515_config(const mcp2515_config_t* config);
static esp_err_t internal_check_mcp2515_in_configuration_mode(const canbus_mcp2515_handle_t handle);
static esp_err_t internal_check_can_frame(const can_frame_t* frame);


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

        .mode = config->spi_cfg.mode,

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
    return mcp2515_send_single_byte_instruction(handle, MCP2515_INSTRUCTION_RESET);
}


esp_err_t canbus_mcp2515_configure_interrupts(canbus_mcp2515_handle_t handle, const mcp2515_interrupt_config_t* config) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // TODO: Check interrupt configuration


    esp_err_t ret = ESP_OK;

    // If interrupts were configured before, undo these configurations first - MCP2515 starts with interrupts disabled by default
    if (handle->interrupt_config.flags != MCP2515_INTERRUPT_DISABLED) {
        // Reset GPIO pin
        ret = gpio_reset_pin(handle->interrupt_config.intr_io_num);

        // TODO: Detach handler
    }

    // Clear configuration
    memset(&handle->interrupt_config, 0, sizeof(handle->interrupt_config));

    // Apply new interrupts configuration
    if (config->flags != MCP2515_INTERRUPT_DISABLED) {
        // Configure the GPIO pin to listen to the MCP2515 interrupt signal
        gpio_config_t gpioConfig = {
            .pin_bit_mask = 1ULL << config->intr_io_num,
            .mode = GPIO_MODE_INPUT, 
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
        };
        ESP_GOTO_ON_ERROR(gpio_config(&gpioConfig), cleanup, CanBusMCP2515LogTag, "%s() Failed to configure GPIO pin %d", __func__, config->intr_io_num);

        // Attach the interrupt handler to the GPIO pin
        // TODO: Chosoe the right interrupt attachmecanism
        //ESP_GOTO_ON_ERROR
        //gpio_isr_register(gpio_num_t gpio_num, gpio_isr_t isr_handler, void *args, int intr_alloc_flags, esp_err_t *err);
        //OR
        //gpio_install_isr_service() and gpio_isr_handler_add() 
        // TODO: Interrupt allocation - https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/api-reference/system/intr_alloc.html
    }

    // Configure the desired interrupts via CANINTE[7:0]
    uint8_t caninte = config->flags;
    ESP_GOTO_ON_ERROR(mcp2515_write_register(handle, MCP2515_CANINTE, caninte), cleanup, CanBusMCP2515LogTag, "%s() Failed to configure MCP2515 [CANINTE]", __func__);

    // Copy config for future reference
    handle->interrupt_config = *config;

    return ret;

cleanup:
    // TODO: Reset gpio config
    // TODO: Detach ISR handler
    // TODO: Disable interrupts on MCP2515 on failure
    
    return ret;
}

esp_err_t canbus_mcp2515_get_interrupt_flags(canbus_mcp2515_handle_t handle, uint8_t* flags) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return mcp2515_read_register(handle, MCP2515_CANINTF, flags);
}


esp_err_t canbus_mcp2515_reset_interrupt_flags(canbus_mcp2515_handle_t handle, mcp2515_interrupts_t flags) {
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Interrupts must have been configured to reset them
    if (handle->interrupt_config.flags == MCP2515_INTERRUPT_DISABLED) {
        return ESP_ERR_INVALID_STATE;
    }

    // Reset interrupt flags via CANINTF[7:0]
    return mcp2515_modify_register(handle, MCP2515_CANINTF, 0, flags);
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

esp_err_t canbus_mcp2515_set_oneshot_mode(canbus_mcp2515_handle_t handle, bool enable) {
    // Configure OSM via CANCTRL (CANCTRL[3])
    uint8_t canctrl = enable ? 0x04 : 0x00;
    return mcp2515_modify_register(handle, MCP2515_CANCTRL, canctrl, 0x03);
}

esp_err_t canbus_mcp2515_configure_bitrate(canbus_mcp2515_handle_t handle, const mcp2515_bit_timing_config_t* bitTimingConfig) {
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
    
    // Take exclusive access of the SPI bus during configuration
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(handle->spi_device_handle, portMAX_DELAY), CanBusMCP2515LogTag, "%s() Unable to acquire SPI bus", __func__);
    
        // Apply bit timing configuration - First CNF3 modify
        esp_err_t err = mcp2515_modify_register(handle, MCP2515_CNF3, cnf3, 0x07);
        if (err == ESP_OK) {
            // Then CNF2 and CNF1 with register address auto increment
            uint8_t cnf2_cnf1[] = { cnf2, cnf1 };
            err = mcp2515_write_registers(handle, MCP2515_CNF2, cnf2_cnf1, sizeof(cnf2_cnf1) / sizeof(cnf2_cnf1[0]));
        }

    // Release access to the SPI bus
    spi_device_release_bus(handle->spi_device_handle);

    return err;
}

esp_err_t canbus_mcp2515_configure_receive_filter(canbus_mcp2515_handle_t handle, const mcp2515_receive_filter_t* filter) {
    // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Options neeed to be specified
    if (filter == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP2515 needs to be in configuration mode to set filtering
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_in_configuration_mode(handle), CanBusMCP2515LogTag, "%s() MCP2515 is not in configuration mode", __func__);

    // Select the mask register associated with the requested filter
    mcp2515_register_t maskRegister = (filter->rxfn == RXF0) || (filter->rxfn == RXF1) ? MCP2515_RXM0SIDH : MCP2515_RXM1SIDH;

    // Select the register to load for the requested filter
    mcp2515_register_t filterRegister;
    switch (filter->rxfn) {
        case RXF0:
            filterRegister = MCP2515_RXF0SIDH;
            break;

        case RXF1:
            filterRegister = MCP2515_RXF1SIDH;
            break;

        case RXF2:
            filterRegister = MCP2515_RXF2SIDH;
            break;

        case RXF3:
            filterRegister = MCP2515_RXF3SIDH;
            break;

        case RXF4:
            filterRegister = MCP2515_RXF4SIDH;
            break;

        case RXF5:
            filterRegister = MCP2515_RXF5SIDH;
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    uint8_t count = 4;
    uint8_t filterSpiBuffer[count];
    uint8_t maskSpiBuffer[count];

    switch (filter->mode) {
        case MCP2515_FILTER_STANDARD_FRAME:
            filterSpiBuffer[0] = (uint8_t) ((filter->filter.standard_frame.id_filter >> 3) & 0x00FF);           // RXFnSIDH
            filterSpiBuffer[1] = (uint8_t)((filter->filter.standard_frame.id_filter & 0x0007) << 3);            // RXFnSIDL
            filterSpiBuffer[2] = (uint8_t) ((filter->filter.standard_frame.data_filter & 0xFF00) >> 8);         // RXFnSEID8
            filterSpiBuffer[3] = (uint8_t) (filter->filter.standard_frame.data_filter & 0x00FF);                // RXFnSEID0

            maskSpiBuffer[0] = (uint8_t) ((filter->filter.standard_frame.id_mask >> 3) & 0x00FF);               // RXMnSIDH
            maskSpiBuffer[1] = (uint8_t) ((filter->filter.standard_frame.id_mask & 0x0007) << 3);               // RXMnSIDL
            maskSpiBuffer[2] = (uint8_t) ((filter->filter.standard_frame.data_mask & 0xFF00) >> 8);             // RXMnEID8
            maskSpiBuffer[3] = (uint8_t) (filter->filter.standard_frame.data_mask & 0x00FF);                    // RXMnEID0
            break;
        
        case MCP2515_FILTER_EXTENDED_FRAME:
            filterSpiBuffer[0] = (uint8_t) ((filter->filter.extended_frame.eid_filter & 0x1FFC0000UL) >> 18);   // RXFnSIDH
            filterSpiBuffer[1] = (uint8_t) (((filter->filter.extended_frame.eid_mask & 0xE00000UL) >> 11) | 8 | 
                                            ((filter->filter.extended_frame.eid_mask & 0x030000UL) >> 16));     // RXFnSIDL
            filterSpiBuffer[2] = (uint8_t) ((filter->filter.extended_frame.eid_filter & 0x00FFUL) >> 8);        // RXFnSEID8
            filterSpiBuffer[3] = (uint8_t) (filter->filter.extended_frame.eid_filter & 0x00FFUL);               // RXFnSEID0

            maskSpiBuffer[0] = (uint8_t) ((filter->filter.extended_frame.eid_mask & 0x1FFC0000UL) >> 18);       // RXMnSIDH
            maskSpiBuffer[1] = (uint8_t) (((filter->filter.extended_frame.eid_mask & 0xE00000UL) >> 11) |  
                                          ((filter->filter.extended_frame.eid_mask & 0x030000UL) >> 16));       // RXMnSIDL
            maskSpiBuffer[2] = (uint8_t) ((filter->filter.extended_frame.eid_mask & 0x00FFUL) >> 8);            // RXMnEID8
            maskSpiBuffer[3] = (uint8_t) (filter->filter.extended_frame.eid_mask & 0x00FFUL);                   // RXMnEID0
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // TODO: Log filter prefix and maskprefix correctly
    ESP_LOGI(CanBusMCP2515LogTag, "Configuring MCP2515 filter RXFnSIDH: 0x%02X, RXFnSIDL: 0x%02X, RXFnSEID8: 0x%02X, RXFnSEID0: 0x%02X | RXMnSIDH: 0x%02X, RXMnSIDL: 0x%02X, RXMnEID8: 0x%02X, RXMnEID0: 0x%02X", filterSpiBuffer[0], filterSpiBuffer[1], filterSpiBuffer[2], filterSpiBuffer[3], maskSpiBuffer[0], maskSpiBuffer[1], maskSpiBuffer[2], maskSpiBuffer[3]);

    // Take exclusive access of the SPI bus during configuration
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(handle->spi_device_handle, portMAX_DELAY), CanBusMCP2515LogTag, "%s() Unable to acquire SPI bus", __func__);
    
        // Apply filter configuration - First filter itself in RXFnSIDH, RXFnSIDL, RXFnEID8, RXFnEID0
        esp_err_t err = mcp2515_write_registers(handle, filterRegister, filterSpiBuffer, count);
        if (err == ESP_OK) {
            // Then filter mask in RXMnSIDH, RXMnSIDL, RXMnEID8, RXMnEID0
            err = mcp2515_write_registers(handle, maskRegister, maskSpiBuffer, count);
        }
        
    // Release access to the SPI bus
    spi_device_release_bus(handle->spi_device_handle);

    return err;
}

esp_err_t canbus_mcp2515_configure_wakeup_lowpass_filter(canbus_mcp2515_handle_t handle, mcp2515_wakeup_lowpass_filter_t filter) {
    // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate configuration
    if ((filter != MCP2515_WAKEUP_LOWPASS_FILTER_DISABLED) && (filter != MCP2515_WAKEUP_LOWPASS_FILTER_ENABLED)) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP2515 needs to be in configuration mode to change CNF3
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_in_configuration_mode(handle), CanBusMCP2515LogTag, "%s() MCP2515 is not in configuration mode", __func__);

    // Configure WAKFIL via CANCTRL[6]
    uint8_t cnf3 = filter == MCP2515_WAKEUP_LOWPASS_FILTER_ENABLED ? 0x40 : 0x00;
    return mcp2515_modify_register(handle, MCP2515_CNF3, cnf3, 0x40);
}

esp_err_t canbus_mcp2515_configure_clkout_sof(canbus_mcp2515_handle_t handle, const mcp2515_clkout_sof_config_t* config) {
    // Options neeed to be specified
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // CLKOUT/SOF requires the pin ot be enabled if it is to beused for any signal
    bool enable_clkoutPin = (config->mode == MCP2515_CLKOUT_PIN_SOF) || (config->mode == MCP2515_CLKOUT_PIN_CLKOUT);

    // Configure CLKOUT pin via CLKEN (CANCTRL[2]) and CLKPRE (CANCTRL[1:0])
    uint8_t canctrl = (enable_clkoutPin ? 0x04 : 0x00) | (config->mode == MCP2515_CLKOUT_PIN_CLKOUT ? config->prescaler & 0x03 : 0);

    // CNF3[7] is CLKOUT/SOF pin enable bit
    uint8_t cnf3 = config->mode == MCP2515_CLKOUT_PIN_SOF ? 0x80 : 0x00;

    // MCP2515 needs to be in configuration mode to change CNF3
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_in_configuration_mode(handle), CanBusMCP2515LogTag, "%s() MCP2515 is not in configuration mode", __func__);

    // Take exclusive access of the SPI bus during configuration
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(handle->spi_device_handle, portMAX_DELAY), CanBusMCP2515LogTag, "%s() Unable to acquire SPI bus", __func__);

        // Apply CLKOUT / SOF configuration - First CANCTRL to enable / disable the CLKOUT/SOF pin
        esp_err_t err = mcp2515_modify_register(handle, MCP2515_CANCTRL, canctrl, 0x07);
        if (err == ESP_OK) {
            // Then CNF3 to enable / disable the CLKOUT/SOF pin
            err = mcp2515_modify_register(handle, MCP2515_CNF3, cnf3, 0x80);
        }

    // Release access to the SPI bus
    spi_device_release_bus(handle->spi_device_handle);

    return err; 
}

esp_err_t canbus_mcp2515_configure_txnrts(canbus_mcp2515_handle_t handle, const mcp2515_txnrts_pins_config_t* config) {
    // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Options neeed to be specified
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP2515 needs to be in configuration mode to change TXRTSCTRL
    ESP_RETURN_ON_ERROR(internal_check_mcp2515_in_configuration_mode(handle), CanBusMCP2515LogTag, "%s() MCP2515 is not in configuration mode", __func__);

    // Configure TXRTSCTRL via TXRTSCTRL[2:0]
    uint8_t txrtsctrl = (config->tx0rts_mode == MCP2515_TXnRTS_PIN_REQUEST_TO_SEND ? 1 : 0) | (config->tx1rts_mode == MCP2515_TXnRTS_PIN_REQUEST_TO_SEND ? 2 : 0) | (config->tx2rts_mode == MCP2515_TXnRTS_PIN_REQUEST_TO_SEND ? 4 : 0);
    return mcp2515_modify_register(handle, MCP2515_TXRTSCTRL, txrtsctrl, 0x07);
}

esp_err_t canbus_mcp2515_get_txnrts(canbus_mcp2515_handle_t handle, uint8_t* txrts) {
    if (txrts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t stagedTxrts;
    esp_err_t err = mcp2515_read_register(handle, MCP2515_TXRTSCTRL, &stagedTxrts);
    if (err == ESP_OK) {
        *txrts = (stagedTxrts >> 3) & 0x07;
    } else {
        *txrts = 0;
    }

    return err;
}

esp_err_t canbus_mcp2515_configure_rxnbf(canbus_mcp2515_handle_t handle,const mcp2515_rxnbf_pins_config_t* config) {
    // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Options neeed to be specified
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // BFPCTRL - BnBFE, BnBFM, BnBFS - We default digital outputs to LOW
    uint8_t bfpctrl = (config->rx0bf == MCP2515_RXnBF_PIN_DISABLED ? 0 : (config->rx0bf == MCP2515_RXnBF_PIN_BUFFER_FULL_INT ? 0x05 : 0x04))  |
                      (config->rx1bf == MCP2515_RXnBF_PIN_DISABLED ? 0 : (config->rx1bf == MCP2515_RXnBF_PIN_BUFFER_FULL_INT ? 0x0A: 0x08));
    esp_err_t err = mcp2515_modify_register(handle, MCP2515_BFPCTRL, bfpctrl, 0x3F);
    if (err == ESP_OK) {
        // TODO: Is this enough to keep the config sync'ed
        handle->rxnbf_config = *config;
    }

    return err;
}

esp_err_t canbus_mcp2515_set_rxnbf(canbus_mcp2515_handle_t handle, mcp2515_rxnbf_pin_t rxnbf, uint32_t level) {
        // Handle must have been initialized, which means we have configured the SPI device
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bfpctrl;
    uint8_t mask;

    // Ensure the requested pin is configured for digital output and calculate correct BFPCTRL and mask
    switch (rxnbf) {
        case MCP2515_RXnBF_PIN_RX0:
            if (handle->rxnbf_config.rx0bf != MCP2515_RXnBF_PIN_DIGITAL_OUTPUT) {
                return ESP_ERR_INVALID_STATE;
            }
            bfpctrl = level ? 0x10: 0;
            mask = 0x10;
        break;

        case MCP2515_RXnBF_PIN_RX1:
            if (handle->rxnbf_config.rx1bf != MCP2515_RXnBF_PIN_DIGITAL_OUTPUT) {
                return ESP_ERR_INVALID_STATE;
            }
            bfpctrl = level ? 0x20 : 0;
            mask = 0x20;
        break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    // Modify the correct bits in BFPCTRL to control the digital output
    return mcp2515_modify_register(handle, MCP2515_BFPCTRL, bfpctrl, mask);
}









inline static uint8_t internal_get_sid10_to_sid3(const uint32_t id) {
    return ((id & CAN_SFF_MASK) >> 3) & 0xFF;
}

inline static uint8_t internal_get_sid2_to_sid0(const uint32_t id) {
    return (id & CAN_SFF_MASK) & 0x07;
}

inline static uint32_t internal_get_eid17_to_eid16(const uint32_t id) {
    return ((id & CAN_EFF_MASK) >> 16) & 0x03;
}

inline static uint32_t internal_get_eid15_to_eid8(const uint32_t id) {
    return ((id & CAN_EFF_MASK) >> 8) & 0xFF;
}

inline static uint32_t internal_get_eid7_to_eid0(const uint32_t id) {
    return (id & CAN_EFF_MASK) & 0xFF;
}



esp_err_t canbus_mcp2515_transmit(canbus_mcp2515_handle_t handle, const can_frame_t* frame, const canbus_mcp2515_transmit_options_t* options) {
    // Validate the frame
    ESP_RETURN_ON_ERROR(internal_check_can_frame(frame), CanBusMCP2515LogTag, "%s() Invalid CAN frame", __func__);

    // TODO: Select the right register - We hard code for now
    mcp2515_TXBn_t effectiveTXn = options->txb == MCP2515_TXB_AUTO ? MCP2515_TXB0 : options->txb;

    // TODO: Do we even need to send RTS ? We might not if the pins were configured to fire the message
    // Select the correct transmit register and RTS instruction to send
    mcp2515_register_t controlRegister;
    mcp2515_register_t idRegister;
    mcp2515_register_t dataRegister;
    mcp2515_instruction_t rtsInstruction;
    switch (effectiveTXn) {
        case MCP2515_TXB0:
            controlRegister = MCP2515_TXB0CTRL;
            idRegister = MCP2515_TXB0SIDH;
            dataRegister = MCP2515_TXB0DO;
            rtsInstruction = MCP2515_INSTRUCTION_RTS_TX0;
            break;
        case MCP2515_TXB1:
            controlRegister = MCP2515_TXB1CTRL;
            idRegister = MCP2515_TXB1SIDH;
            dataRegister = MCP2515_TXB1DO;
            rtsInstruction = MCP2515_INSTRUCTION_RTS_TX1;
            break;
        case MCP2515_TXB2:
            controlRegister = MCP2515_TXB2CTRL;
            idRegister = MCP2515_TXB2SIDH;
            dataRegister = MCP2515_TXB2DO;
            rtsInstruction = MCP2515_INSTRUCTION_RTS_TX2;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // Prepare the buffer to load the frame and transmit
    bool isExtendedFrame = frame->options & CAN_FRAME_OPTION_EXTENDED;

    // Allocate a buffer large enough to contain the entire frame with data if present
    //                     | TXB0CTRL | TXBnSIDH TXBnSIDL | TXBnDLC |   TXBnEID8 TXBnEID0   |
    const uint8_t cmdCount =    1     +         2         +    1    + isExtendedFrame ? 2 : 0;
    uint8_t transmitBuffer[cmdCount + frame->dlc];




    // TODO: priority = 0 to 3 (3 highest) and setTREQ
    // TODO
    uint8_t priority = 3;
    bool setTREQ = false; // TODO: Decide if we are setting the flag to send or reliying on pins or relying on SPI RTS command
    
    // TODO: Decide this
    // TXBnCTRL - TXREQ[3] TXP[1:0]
    transmitBuffer[0] = priority | setTREQ ? 0x04 : 0;

    // Statndard ID - TXBnSIDH and TXBnSIDL
    transmitBuffer[1] = internal_get_sid10_to_sid3(frame->id);
    transmitBuffer[2] = internal_get_sid2_to_sid0(frame->id) << 5 | (isExtendedFrame ? (0x08 | internal_get_eid17_to_eid16(frame->id)) : 0);

    // Extended ID configuration
    if (isExtendedFrame) {
        // TXBnEID8 and TXBnEID0
        transmitBuffer[3] = internal_get_eid15_to_eid8(frame->id); 
        transmitBuffer[4] = internal_get_eid7_to_eid0(frame->id);
    }

    // Data length - TXBnDLC
    transmitBuffer[isExtendedFrame ? 5 : 3] = frame->dlc;

    // Copy data
    if (frame->dlc > 0) {
        // TXBnDm
        memcpy(&transmitBuffer[cmdCount], frame->data, frame->dlc);
    }

    // Log the prepared buffer for debugging
    ESP_LOG_BUFFER_HEXDUMP(CanBusMCP2515LogTag, transmitBuffer, cmdCount + frame->dlc, ESP_LOG_INFO);


    // TODO Make sure  TXREQ (TXBnCTRL[3]) is clear before writing to the register
    bool readyToTransmit = false;
    esp_err_t err = ESP_OK;
    do {
        uint8_t txbnctrl = 0;
        err = mcp2515_read_register(handle, controlRegister, &txbnctrl);
        if (err != ESP_OK) {
            break;
        }

        readyToTransmit = (txbnctrl & 0x08) == 0;

    } while (!readyToTransmit);

    if (err == ESP_OK) {

        // Take exclusive access of the SPI bus while loading transmit buffers
        ESP_RETURN_ON_ERROR(spi_device_acquire_bus(handle->spi_device_handle, portMAX_DELAY), CanBusMCP2515LogTag, "%s() Unable to acquire SPI bus", __func__);

            // Configure TXnIE via CANINTE
            // TODO: configure only once - we should not be reconfiguring interrupts for each frame
            esp_err_t err = mcp2515_write_register(handle, MCP2515_CANINTE, 0);
            if (err == ESP_OK) {
                // TODO: Load with more performant methods to reduce SPI size
                // TODO: For extended frames, we can load all registers with a single auto incrementing 'WRITE' instruction
                err = mcp2515_write_registers(handle, controlRegister, transmitBuffer, cmdCount);
                if (err == ESP_OK) {
                    if (frame->dlc > 0) {
                        err = mcp2515_write_registers(handle, dataRegister, &transmitBuffer[cmdCount], frame->dlc);
                    }
                    
                    if (err == ESP_OK) {
                        // Send RTS to transmit the frame
                        // TODO: Make this configurable
                        err = mcp2515_send_single_byte_instruction(handle, rtsInstruction);
                    }
                }
            }

        // Release access to the SPI bus
        spi_device_release_bus(handle->spi_device_handle);

    }

    // TODO: Wait for TXREQ bit (TXBnCTRL[3]) to become 0 indicating it was transmitted
    
    return err;
}

esp_err_t canbus_mcp2515_set_transmit_abort(canbus_mcp2515_handle_t handle, mcp2515_transmit_behavior_t mode) {
    // validate configuration
    if ((mode != MCP2515_TRANSMIT_BEHAVIOR_NORMAL) && (mode != MCP2515_TRANSMIT_BEHAVIOR_ABORT)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t canctrl = mode == MCP2515_TRANSMIT_BEHAVIOR_ABORT ? 0x40 : 0x00;
    return mcp2515_modify_register(handle, MCP2515_CANCTRL, canctrl, 0x40); 
}



esp_err_t canbus_mcp2515_get_transmit_error_count(canbus_mcp2515_handle_t handle, uint8_t* count) {
    return mcp2515_read_register(handle, MCP2515_TEC, count);
}

esp_err_t canbus_mcp2515_get_receive_error_count(canbus_mcp2515_handle_t handle, uint8_t* count) {
    return mcp2515_read_register(handle, MCP2515_REC, count);
}

esp_err_t canbus_mcp1515_get_error_flags(canbus_mcp2515_handle_t handle, uint8_t* flags) {
    return mcp2515_read_register(handle, MCP2515_EFLG, flags);
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


    // Prepare the command buffer with the provided payload
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

esp_err_t mcp2515_send_single_byte_instruction(canbus_mcp2515_handle_t handle, const mcp2515_instruction_t instruction) {
    // TODO: Validate in debug the instruction is a single byte instruction
    if (handle->spi_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

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






static esp_err_t internal_check_mcp2515_config(const mcp2515_config_t* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP2515 data sheet indicates the SPI clock is 10MHz maximum
    if (config->spi_cfg.clock_speed_hz > 10 * 1000000) {
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

static esp_err_t internal_check_can_frame(const can_frame_t* frame) {
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Standard frames must have an ID within bounds
    if (((frame->options & CAN_FRAME_OPTION_EXTENDED) == 0) && ((frame->id & 0x000007FFUL) != frame->id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Extended frames must have an ID within bounds
    if ((frame->options & CAN_FRAME_OPTION_EXTENDED) && ((frame->id & 0x1FFFFFFFUL) != frame->id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Remote Transmission Requests must have no data
    if ((frame->options & CAN_FRAME_OPTION_RTR) && (frame->dlc > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Data length must be within bounds
    if (frame->dlc > CAN_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}