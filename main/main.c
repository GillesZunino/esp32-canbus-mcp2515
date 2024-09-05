// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include "sdkconfig.h"

#include <freertos/FreeRTOS.h>
#include <esp_check.h>
#include <driver/gpio.h>

#include "can.h"
#include "canbus_mcp2515.h"
#include "canbus_mcp2515_utils.h"

const char* TAG = "mcp2515_main";

//
// NOTE: For maximum performance, prefer IO MUX  over GPIO Matrix routing
//  * When using GPIO Matrix routing, the SPI bus speed is limited to 20 MHz and it may be necessary to adjust spi_device_interface_config_t::input_delay_ns
//  * See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#gpio-matrix-routing
//
const spi_host_device_t SPI_HOSTID = SPI2_HOST;

const gpio_num_t CS_PIN = GPIO_NUM_9;
const gpio_num_t SCLK_PIN = GPIO_NUM_10;
const gpio_num_t MOSI_PIN = GPIO_NUM_11;
const gpio_num_t MISO_PIN = GPIO_NUM_12;


canbus_mcp2515_handle_t can_mcp2515_handle = NULL;


void app_main(void) {
    // Configure SPI bus to communicate with MCP2515
    spi_bus_config_t spiBusConfig = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN,

        .data2_io_num = GPIO_NUM_NC,
        .data3_io_num = GPIO_NUM_NC,

        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOSTID, &spiBusConfig, SPI_DMA_CH_AUTO));

    // Initialize the MCP2515 driver
    mcp2515_config_t mcp2515InitConfig = {
        .spi_cfg = {
            .host_id = SPI_HOSTID,

            // NOTE: Prefer mode = 0 [(0,0)] because it avoids one known issue with the MCP2515 - See https://ww1.microchip.com/downloads/en/DeviceDoc/80000179H.pdf
            .mode = 0,

            .clock_source = SPI_CLK_SRC_DEFAULT,
            .clock_speed_hz = 10 * 1000000,

            // NOTE: This may need adjustment when using GPIO Matrix routing
            //.input_delay_ns = 50,

            .spics_io_num = CS_PIN,
            .queue_size = 8
        }
    };
    ESP_LOGI(TAG, "Initialize MCP2515 driver");
    ESP_ERROR_CHECK(canbus_mcp2515_init(&mcp2515InitConfig, &can_mcp2515_handle));

    // Reset MCP2515 after power on - This automatically sets the MCP2515 in configuration mode
    ESP_LOGI(TAG, "Reset MCP2515 after power on");
    ESP_ERROR_CHECK(canbus_mcp2515_reset(can_mcp2515_handle));

    // Configure low pass fitler for wake-up
    ESP_LOGI(TAG, "Configure MCP2515 Wake-up low pass filter");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_wakeup_lowpass_filter(can_mcp2515_handle, MCP2515_WAKEUP_LOWPASS_FILTER_ENABLED));

    // Configure the MCP2515 CLKOUT pin
    mcp2515_clkout_sof_config_t clkoutSofConfig = {
        .mode = MCP2515_CLKOUT_PIN_OFF,
        .prescaler = MCP2515_CLKOUT_DIVIDER_4
    };
    ESP_LOGI(TAG, "Configure MCP2515 CLKOUT/SOF pin");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_clkout_sof(can_mcp2515_handle, &clkoutSofConfig));

    // Configure MCP2515 TXnRST pins behavior
    mcp2515_txnrts_pins_config_t txnrtsConfig = {
        .tx0rts_mode = MCP2515_TXnRTS_PIN_DIGITAL_INPUT,
        .tx1rts_mode = MCP2515_TXnRTS_PIN_DIGITAL_INPUT,
        .tx2rts_mode = MCP2515_TXnRTS_PIN_DIGITAL_INPUT
    };
    ESP_LOGI(TAG, "Configure MCP2515 TXnRST pins to digital input");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_txnrts(can_mcp2515_handle, &txnrtsConfig));

    // Configure MCP2515 RXnBF pins behavior
    mcp2515_rxnbf_pins_config_t rxnbfConfig = {
        .rx0bf = MCP2515_RXnBF_PIN_DIGITAL_OUTPUT,
        .rx1bf = MCP2515_RXnBF_PIN_DIGITAL_OUTPUT
    };
    ESP_LOGI(TAG, "Configure MCP2515 RXnBF pins to digital output");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_rxnbf(can_mcp2515_handle, &rxnbfConfig));

    // Configure MCP2515 CAN bit timings - See CAN bit timing calculator in README
    mcp2515_bit_timing_config_t bitTimingConfig = {
        .bit_rate_prescaler = 1,
        .sjw = 1,
        .btl_mode = MCP2515_BTL_MODE_EXPLICIT,
        .sample_mode = MCP2515_SAMPLING_MODE_ONCE,
        .propagation_seg = 1,
        .phase_seg1 = 1,
        .phase_seg2 = 1
    };
    ESP_LOGI(TAG, "Configure MCP2515 bit rate");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_bitrate(can_mcp2515_handle, &bitTimingConfig));

    // Set a receive filter RXF0 - Applies to Standard frames
    mcp2515_receive_filter_t rxf0StandardFrameFilter = {
        .rxfn = RXF0,
        .mode = MCP2515_FILTER_STANDARD_FRAME,
        .filter.standard_frame = {
            .id_filter = 0x123,     // Accept only frames with this ID ...
            .id_mask = 0x7FF,       // Filter on all 11 bits of Standard ID
            .data_filter = 0x0102,  // ... and this data (frame byte 0 = 0x01 and frame byte 1 = 0x02)
            .data_mask = 0xFFFF     // Filter on all 16 bits of data
        }
    };
    ESP_LOGI(TAG, "Configure MCP2515 Standard frame filter RXF0");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_receive_filter(can_mcp2515_handle, &rxf0StandardFrameFilter));

    // Set a receive filter RXF2 - Applies to Extended frames
    mcp2515_receive_filter_t rxf2ExtendedFrameFilter = {
        .rxfn = RXF2,
        .mode = MCP2515_FILTER_EXTENDED_FRAME,
        .filter.extended_frame = {
            .eid_filter = 0x1234567,     // Accept only frames with this ID ...
            .eid_mask = CAN_EFF_MASK     // Filter on all 29 bits of Extended ID
        }
    };
    ESP_LOGI(TAG, "Configure MCP2515 Extended frame filter RXF2");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_receive_filter(can_mcp2515_handle, &rxf2ExtendedFrameFilter));

    // Configure interrupts
    mcp2515_interrupt_config_t interruptsConfig = {
        .flags = MCP2515_INTERRUPT_DISABLED
    };
    ESP_LOGI(TAG, "Configure MCP2515 interrupts");
    ESP_ERROR_CHECK(canbus_mcp2515_configure_interrupts(can_mcp2515_handle, &interruptsConfig));

    // Standard CAN frame which will pass filtering
    can_frame_t filterdInStandardFrame = {
        // TODO: Make this work
        .id = 0x123,
        .dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // CAN frame which will be filtered out
    can_frame_t filterdOutStandardFrame = {
        // TODO: Make this work
        .id = 0x456,
        .dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // Extended CAN frame which will pass filtering
    can_frame_t filterdInExtendedFrame = {
        // TODO: Make this work
        .options = CAN_FRAME_OPTION_EXTENDED,
        .id = 0x1234567,
        .dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // Extended CAN frame which will be filtered out
    can_frame_t filterdOutExtendedFrame = {
        // TODO: Make this work
        .options = CAN_FRAME_OPTION_EXTENDED,
        .id = 0x7654321,
        .dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // Configure 'special receive mode' for debugging
    ESP_LOGI(TAG, "Configure MCP2515 'special receive mode'");
    ESP_ERROR_CHECK(canbus_mcp2515_set_special_receive(can_mcp2515_handle, true));

    // Enable rollover mode
    ESP_LOGI(TAG, "Configure MCP2515 receive rollover");
    ESP_ERROR_CHECK(canbus_mcp2515_set_receive_rollover(can_mcp2515_handle, true));

    // Set the MCP2515 in loopback mode for testing with one node only
    ESP_ERROR_CHECK(canbus_mcp2515_set_mode(can_mcp2515_handle, MCP2515_MODE_LOOPBACK, 50));

    // Send a few frames to see the effect of receive filtering
    const uint16_t NumberOfFramesToSend = 20;
    const TickType_t DelayBetweenFrames = pdMS_TO_TICKS(1000);

    uint8_t framesSent = 0;

    // Used to toggle digitial outputs (RXnBF) HIGH and LOW
    uint8_t rx0Counter = 0;
    uint8_t rx1Counter = 0;
    do {
        // Send a frame if necessary
        if (framesSent <= NumberOfFramesToSend) {
            can_frame_t* pCanFrame = NULL;

            framesSent++;

            // Alternate between standard and extended frames (filtered in or out)
            if (framesSent % 9 == 0) {
                ESP_LOGI(TAG, "Preparing to send FILTERED IN EXTENDED frame");
                pCanFrame = &filterdInExtendedFrame;
            } else {
                if (framesSent % 3 == 0) {
                    ESP_LOGI(TAG, "Preparing to send FILTERED OUT EXTENDED frame");
                    pCanFrame = &filterdOutExtendedFrame;
                } else {
                    if (framesSent % 4 == 0) {
                        ESP_LOGI(TAG, "Preparing to send FILTERED IN STANDARD frame");
                        pCanFrame = &filterdInStandardFrame;
                    } else {
                        ESP_LOGI(TAG, "Preparing to send FILTERED OUT STANDARD frame");
                        pCanFrame = &filterdOutStandardFrame;
                    }
                }
            }

            pCanFrame->data[0] = framesSent;

            mcp2515_TXBn_t effectiveTransmitRegister = MCP2515_TXB_NONE;
            mcp2515_transmit_options_t sendOptions = {
                .txb = MCP2515_TXB_AUTO,
                .priority = MCP1515_TRANSMIT_PRIORITY_MEDIUM 
            };

            ESP_LOGI(TAG, "Sending frame with index %d", framesSent);
            esp_err_t err = canbus_mcp2515_transmit(can_mcp2515_handle, pCanFrame, &sendOptions, &effectiveTransmitRegister);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error sending frame with index %d: %d", framesSent, err);
            }
        }

        // Retrieve transmission status from MCP2515 
        canintf_t canintf;
        ESP_ERROR_CHECK(canbus_mcp2515_get_interrupt_flags(can_mcp2515_handle, &canintf));
        MCP2515_LOG_CANINTF(TAG, canintf.flags, ESP_LOG_INFO);

        // If a transmit register becomes empty (sucessful transmission), clear its status
        if (canintf.bits.tx0if || canintf.bits.tx1if || canintf.bits.tx2if) {
            if (canintf.bits.tx0if) {
                ESP_LOGI(TAG, "Clearing TX0IF in CANINTF");
                ESP_ERROR_CHECK(canbus_mcp2515_reset_interrupt_flags(can_mcp2515_handle, MCP2515_INTERRUPT_TX0_MSG_SENT));
            }
            
            if (canintf.bits.tx1if) {
                ESP_LOGI(TAG, "Clearing TX1IF in CANINTF");
                ESP_ERROR_CHECK(canbus_mcp2515_reset_interrupt_flags(can_mcp2515_handle, MCP2515_INTERRUPT_TX1_MSG_SENT));
            }

            if (canintf.bits.tx2if) {
                ESP_LOGI(TAG, "Clearing TX2IF in CANINTF");
                ESP_ERROR_CHECK(canbus_mcp2515_reset_interrupt_flags(can_mcp2515_handle, MCP2515_INTERRUPT_TX2_MSG_SENT));
            }
        }

        // If a receive register becomes available for reading, retrieve it now
        if (canintf.bits.rx0if || canintf.bits.rx1if) {
            can_frame_t receivedFrameRxb0;
            mcp2515_receive_filter_hit_t filtersHitRxb0;

            can_frame_t receivedFrameRxb1;
            mcp2515_receive_filter_hit_t filtersHitRxb1;

            // TODO: Demonstrate how to read messages in order - Reading RXB0 then RXB1
            if (canintf.bits.rx0if) {
                ESP_LOGI(TAG, "Reading frame from RXB0");
                ESP_ERROR_CHECK(canbus_mcp2515_receive(can_mcp2515_handle, MCP2515_RECEIVE_RXB0, &receivedFrameRxb0, &filtersHitRxb0));

                ESP_LOGI(TAG, "Clearing RX0IF in CANINTF");
                ESP_ERROR_CHECK(canbus_mcp2515_reset_interrupt_flags(can_mcp2515_handle, MCP2515_INTERRUPT_RX0_MSG_RECEIVED));
            }
            
            if (canintf.bits.rx1if) {
                ESP_LOGI(TAG, "Reading frame from RXB1");
                ESP_ERROR_CHECK(canbus_mcp2515_receive(can_mcp2515_handle, MCP2515_RECEIVE_RXB1, &receivedFrameRxb1, &filtersHitRxb1));

                ESP_LOGI(TAG, "Clearing RX1IF in CANINTF");
                ESP_ERROR_CHECK(canbus_mcp2515_reset_interrupt_flags(can_mcp2515_handle, MCP2515_INTERRUPT_RX1_MSG_RECEIVED));            
            }
        }

        // Retrieve transmit / receive error count
        uint8_t transmitErrorCount = 0;
        uint8_t receiveErrorCount = 0;
        eflg_t eflg = {0};
        ESP_ERROR_CHECK(canbus_mcp2515_get_transmit_error_count(can_mcp2515_handle, &transmitErrorCount));
        ESP_ERROR_CHECK(canbus_mcp2515_get_receive_error_count(can_mcp2515_handle, &receiveErrorCount));
        ESP_ERROR_CHECK(canbus_mcp1515_get_error_flags(can_mcp2515_handle, &eflg));
        ESP_LOGI(TAG, "MCP2515 REC: %d - TEC: %d", receiveErrorCount, transmitErrorCount);
        MCP2515_LOG_EFLG(TAG, eflg.flags, ESP_LOG_INFO);

        // Abort tranmissions for all TX buffers
        // ESP_LOGI(TAG, "Configure MCP2515 interrupts");
        // ESP_ERROR_CHECK(canbus_mcp2515_set_transmit_abort(can_mcp2515_handle, MCP2515_TRANSMIT_BEHAVIOR_ABORT));

        // Retrieve digital inputs state via TXnRTS pins
        uint8_t txnrts = 0;
        ESP_ERROR_CHECK(canbus_mcp2515_get_txnrts(can_mcp2515_handle, &txnrts));
        ESP_LOGI(TAG, "Digital Inputs - TX0RTS: %s | TX1RTS: %s | TX2RTS: %s", (txnrts & MCP2515_TXnRTS_PIN_TX0) ? "HIGH" : "LOW", (txnrts & MCP2515_TXnRTS_PIN_TX1) ? "HIGH" : "LOW", (txnrts & MCP2515_TXnRTS_PIN_TX2) ? "HIGH" : "LOW");

        // Toggle digital outputs via RXnBF
        rx0Counter = (rx0Counter + 1) % 2;  // RX0BF goes from HIGH to LOW on every loop        
        rx1Counter = (rx1Counter + 1) % 6;  // RX1BF goes from HIGH to LOW on every three loops
        bool rx1State = rx1Counter < 3;    
        ESP_ERROR_CHECK(canbus_mcp2515_set_rxnbf(can_mcp2515_handle, MCP2515_RXnBF_PIN_RX0, rx0Counter));
        ESP_ERROR_CHECK(canbus_mcp2515_set_rxnbf(can_mcp2515_handle, MCP2515_RXnBF_PIN_RX1, rx1State));
        ESP_LOGI(TAG, "Digital Outputs - RX0BF: %s | RX1BF: %s", rx0Counter ? "HIGH" : "LOW", rx1State ? "HIGH" : "LOW");

        vTaskDelay(DelayBetweenFrames);
    } while (true);

    // Shutdown MCP2515 driver and SPI bus
    ESP_ERROR_CHECK(canbus_mcp2515_free(can_mcp2515_handle));
    ESP_ERROR_CHECK(spi_bus_free(SPI_HOSTID));
}
