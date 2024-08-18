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

    // Configure MCP2515 in loopback mode
    const TickType_t ModeChangeDelay = pdMS_TO_TICKS(20);
    const mcp2515_mode_t DesiredMCP1515Mode = MCP2515_MODE_LOOPBACK;
    ESP_LOGI(TAG, "Set MCP2515 in mode %d", DesiredMCP1515Mode);
    ESP_ERROR_CHECK(canbus_mcp2515_set_mode(can_mcp2515_handle, DesiredMCP1515Mode, ModeChangeDelay));

    // Read mode and log
    mcp2515_mode_t mcp2515Mode;
    ESP_ERROR_CHECK(canbus_mcp2515_get_mode(can_mcp2515_handle, &mcp2515Mode));
    ESP_LOGI(TAG, "MCP2515 is in mode %d", mcp2515Mode);

    const TickType_t DelayBetweenFrames = pdMS_TO_TICKS(100);
    do {
         vTaskDelay(DelayBetweenFrames);
    } while (true);

    // Shutdown MCP2515 driver and SPI bus
    ESP_ERROR_CHECK(canbus_mcp2515_free(can_mcp2515_handle));
    ESP_ERROR_CHECK(spi_bus_free(SPI_HOSTID));
}
