// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include "sdkconfig.h"

#include <freertos/FreeRTOS.h>
#include <esp_check.h>
#include <driver/gpio.h>

#include "can.h"
#include "canbus_mcp2515.h"

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

    // Configure the MCP2515 CLKOUT pin
    mcp2515_clkout_config_t clkoutConfig = {
        .enable = true,
        .prescaler = MCP2515_CLKOUT_DIVIDER_4
    };
    ESP_LOGI(TAG, "Configure MCP2515 CLKOUT pin");
    ESP_ERROR_CHECK(canbus_mcp2515_set_clkout(can_mcp2515_handle, &clkoutConfig));

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
    ESP_ERROR_CHECK(canbus_mcp2515_set_bitrate(can_mcp2515_handle, &bitTimingConfig));

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
    ESP_ERROR_CHECK(canbus_mcp2515_set_receive_filter(can_mcp2515_handle, &rxf0StandardFrameFilter));

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
    ESP_ERROR_CHECK(canbus_mcp2515_set_receive_filter(can_mcp2515_handle, &rxf2ExtendedFrameFilter));


    // ESP_ERROR_CHECK(canbus_mcp2515_set_isr_handler());

    // Standard CAN frame which will pass filtering
    can_frame_t filterdInStandardFrame = {
        .can_id = 0x123,
        .can_dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // CAN frame which will be filtered out
    can_frame_t filterdOutStandardFrame = {
        // TODO: Make this work
        .can_id = 0x456,
        .can_dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // Extended CAN frame which will pass filtering
    can_frame_t filterdInExtendedFrame = {
        // TODO: Make this work
        .can_id = 0x1234567,
        .can_dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // Extended CAN frame which will be filtered out
    can_frame_t filterdOutExtendedFrame = {
        // TODO: Make this work
        .can_id = 0x7654321,
        .can_dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    // Set the MCP2515 in loopback mode for testing with one node only
    ESP_ERROR_CHECK(canbus_mcp2515_set_mode(can_mcp2515_handle, MCP2515_MODE_LOOPBACK, 50));

    // Send a few frames to see the effect of receive filtering
    const uint16_t NumberOfFramesToSend = 20;
    const TickType_t DelayBetweenFrames = pdMS_TO_TICKS(1000);

    uint8_t framesSent = 0;
    do {
        // Send a frame if necessary
        if (framesSent <= NumberOfFramesToSend) {
            can_frame_t* pCanFrame = NULL;

            framesSent++;

            // Alternate between standard and extended frames (filtered in or out)
            if (framesSent % 9 == 0) {
                pCanFrame = &filterdInExtendedFrame;
            } else {
                if (framesSent % 3 == 0) {
                    pCanFrame = &filterdOutExtendedFrame;
                } else {
                    if (framesSent % 4 == 0) {
                        pCanFrame = &filterdInStandardFrame;
                    } else {
                        pCanFrame = &filterdOutStandardFrame;
                    }
                }
            }

            pCanFrame->data[0] = framesSent;
            // esp_err_t err = canbus_mcp2515_send(can_mcp2515_handle, pCanFrame);
            // if (err != ESP_OK) {
            //     ESP_LOGE(TAG, "Error sending frame with index %d: %d", framesSent, err);
            // }
        }

        // Retrieve transmit/ receive error count
        uint8_t transmitErrorCount = 0;
        uint8_t receiveErrorCount = 0;
        ESP_ERROR_CHECK(canbus_mcp2515_get_transmit_error_count(can_mcp2515_handle, &transmitErrorCount));
        ESP_ERROR_CHECK(canbus_mcp2515_get_receive_error_count(can_mcp2515_handle, &receiveErrorCount));
        ESP_LOGI(TAG, "MCP2515 Receive Error Count %d - Transmit Error Count %d", receiveErrorCount, transmitErrorCount);

        vTaskDelay(DelayBetweenFrames);
    } while (true);

    // Shutdown MCP2515 driver and SPI bus
    ESP_ERROR_CHECK(canbus_mcp2515_free(can_mcp2515_handle));
    ESP_ERROR_CHECK(spi_bus_free(SPI_HOSTID));
}
