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
// NOTE: For maximum performance, choose "native" SPI pins to avoid the default GPIO matrix routing
//
const spi_host_device_t SPI_HOSTID = SPI2_HOST;

const gpio_num_t CS_PIN = GPIO_NUM_10;
const gpio_num_t SCLK_PIN = GPIO_NUM_11;
const gpio_num_t MOSI_PIN = GPIO_NUM_12;
const gpio_num_t MISO_PIN = GPIO_NUM_13;


canbus_mcp2515_handle_t can_mcp2515_handle = NULL;


void app_main(void) {
    // Configure SPI bus to communicate with MCP2515
    spi_bus_config_t spiBusConfig = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN,

        .data2_io_num = GPIO_NUM_NC,
        .data3_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,

        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOSTID, &spiBusConfig, SPI_DMA_CH_AUTO));

    // Initialize the MCP2515 driver
    mcp2515_config_t mcp2515InitConfig = {
        .spi_cfg = {
            .host_id = SPI_HOSTID,
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .clock_speed_hz = 10 * 1000000,
            .spics_io_num = CS_PIN
        }
    };
    ESP_ERROR_CHECK(canbus_mcp2515_init(&mcp2515InitConfig, &can_mcp2515_handle));

    // Reset MCP2515 after power on - This automatically sets the MCP2515 in configuration mode
    ESP_ERROR_CHECK(canbus_mcp2515_reset(can_mcp2515_handle));

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
    ESP_ERROR_CHECK(canbus_mcp2515_set_bitrate(can_mcp2515_handle, &bitTimingConfig));

    // Set the MCP2515 in loopback mode for testing with one node only
    ESP_ERROR_CHECK(canbus_mcp2515_set_mode(can_mcp2515_handle, MCP2515_MODE_NORMAL, 50));

    // Transmit a few CAN frames
    can_frame_t frame = {
        .can_id = 0x123,
        .can_dlc = 8,
        .data = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }
    };

    const uint16_t NumberOfFramesToSend = 10;
    const TickType_t DelayBetweenFrames = pdMS_TO_TICKS(2);

    for (uint16_t frameCount = 1; frameCount <= NumberOfFramesToSend; frameCount++) {
        // Update frame data with frame number
        frame.data[1] = frameCount;

        // Send frame
        esp_err_t err = canbus_mcp2515_send(can_mcp2515_handle, &frame);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error sending frame with index %d: %d", frameCount, err);
        }

        vTaskDelay(DelayBetweenFrames);
    }

    // Shutdown MCP2515 driver and SPI bus
    ESP_ERROR_CHECK(canbus_mcp2515_free(can_mcp2515_handle));
    ESP_ERROR_CHECK(spi_bus_free(SPI_HOSTID));
}
