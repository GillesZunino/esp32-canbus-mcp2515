// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <driver/spi_master.h>

#include "can.h"
#include "canbus_mcp2515_types.h"


#ifdef __cplusplus
extern "C" {
#endif


struct canbus_mcp2515;
typedef struct canbus_mcp2515* canbus_mcp2515_handle_t; ///< Handle to a MCP2515 device


/**
 * @brief Configuration of the SPI bus for MCP2515 device.
 */
typedef struct mcp2515_spi_config {
    spi_host_device_t host_id;          ///< SPI bus ID. Which buses are available depends on the specific chip
    uint8_t mode;                       ///< SPI mode. MCP2515 supports mode = 0 for (0,0) and mode = 3 for (1,1)
    spi_clock_source_t clock_source;    ///< Select SPI clock source, `SPI_CLK_SRC_DEFAULT` by default
    int clock_speed_hz;                 ///< SPI clock speed in Hz. Derived from `clock_source`
    int input_delay_ns;                 ///< Maximum data valid time of slave. The time required between SCLK and MISO
    int spics_io_num;                   ///< CS GPIO pin for this device, or `GPIO_NUM_NC` (-1) if not used
    int queue_size;                     ///< SPI transaction queue size. See 'spi_device_queue_trans()'
} mcp2515_spi_config_t;

/**
 * @brief Configuration of MCP2515 device.
 */
typedef struct mcp2515_config {
    mcp2515_spi_config_t spi_cfg;
} mcp2515_config_t;

/**
 * @brief MCP1515 BTL mode.
 */
typedef enum canbus_mcp2515_btl_mode {
    MCP2515_BTL_MODE_EXPLICIT = 1,                  ///< Phase Segment 2 length explicitely configured
    MCP2515_BTL_MODE_GREATER_THAN_PS1_AND_IPT = 0   ///< Phase Segment 2 length = max(Phase Segment 1, IPT)
} canbus_mcp2515_blt_mode_t;

/**
 * @brief MCP2515 sampling mode.
 */
typedef enum canbus_mcp2515_sampling_mode {
    MCP2515_SAMPLING_MODE_ONCE = 0,     ///< Signal is sampled once
    MCP2515_SAMPLING_MODE_THREE = 1     ///< Signal is sampled three times
} canbus_mcp2515_sampling_mode_t;

/**
 * @brief MCP2515 Bit timing configuration.
 */
typedef struct mcp2515_bit_timing_config {
    uint8_t bit_rate_prescaler;                 // 1...64
    uint8_t sjw;                                // 1...4
    canbus_mcp2515_blt_mode_t btl_mode;
    canbus_mcp2515_sampling_mode_t sample_mode;
    uint8_t propagation_seg;                    // 1..8
    uint8_t phase_seg1;                         // 1..8
    uint8_t phase_seg2;                         // 1..8
} mcp2515_bit_timing_config_t;


/**
 * @brief MCP2515 Receive filters.
 */
typedef enum {
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
} mcp2515_RXFn_t;

/**
 * @brief MCP2515 filter / mask mode.
 */
typedef enum {
    MCP2515_FILTER_STANDARD_FRAME = 1,
    MCP2515_FILTER_EXTENDED_FRAME = 2
} mcp2515_filter_mode_t;


/**
 * @brief MCP2515 receive filter.
 */
typedef struct mcp2515_receive_filter {
    mcp2515_RXFn_t rxfn;                ///< Receive filter to configure (RFX0 ... RFX5)
    mcp2515_filter_mode_t mode;         ///< Filter applies to standard frames only or extended frames only             
    union {                             ///< Filter
        struct {
            uint16_t id_filter;         ///< Standard ID filter. Only the least significant 11 bits are used
            uint16_t id_mask;           ///< Standard ID mask. Only the lesat significant 11 bits are used
            uint16_t data_filter;       ///< Data filter. The MSB applies to frame data[0] and LSB applied to frame data[1]
            uint16_t data_mask;         ///< Data mask.
        } standard_frame;

        struct {
            uint32_t eid_filter;        ///< Exrtended ID filter. Only the least significant 29 bits are used
            uint32_t eid_mask;          ///< Extended ID mask. Only the least significant 29 bits are used
        } extended_frame;
    } filter __attribute__((__packed__));
} mcp2515_receive_filter_t;



/**
 * @brief MCP2515 modes of operation.
 * @note On power on, the MCP2515 automatically enters configuration mode.
 */
typedef enum {
    MCP2515_MODE_NORMAL     = 0x00,
    MCP2515_MODE_SLEEP      = 0x01,
    MCP2515_MODE_LOOPBACK   = 0x02,
    MCP2515_MODE_LISTENONLY = 0x03,
    MCP2515_MODE_CONFIG     = 0x04
} mcp2515_mode_t;





/**
 * @brief Installs the MCP2515 driver and get its handle.
 *
 * @param      config   Configuration for the ESSL SPI device
 * @param[out] handle   Pointer to a memory location which receives the handle
 * @return
 *        - ESP_OK:                On success
 *        - ESP_ERR_NO_MEM:        Memory exhausted
 *        - ESP_ERR_INVALID_STATE: SPI driver is not initialized
 *        - ESP_ERR_INVALID_ARG:   Invalid configuration
 */
esp_err_t canbus_mcp2515_init(const mcp2515_config_t* config, canbus_mcp2515_handle_t* handle);

/**
 * @brief Release the MCP2515 driver and free the memory used by the device.
 *
 * @param handle    Handle of the MCP2515 device
 * @return
 *        - ESP_OK:                On success
 *        - ESP_ERR_INVALID_STATE: MCP2515 device is not in use
 * @note The device will be reset before releasing it
 *  */
esp_err_t canbus_mcp2515_free(canbus_mcp2515_handle_t handle);

/**
 * @brief Reset the MCP2515 device.
 *
 * @param handle    Handle of the MCP2515 device
 * @note The device will automatically enter configuration mode after reset
 * @return
 *        - ESP_OK:                On success
 *        - ESP_ERR_INVALID_STATE: MCP2515 device is not in use
 */
esp_err_t canbus_mcp2515_reset(canbus_mcp2515_handle_t handle);

/**
 * @brief Get the operation mode of the MCP2515 device.
 * @param handle    Handle of the MCP2515 device
 * @param pMode     Pointer to a memory location which receives the mode
 * @return 
 *       - ESP_OK:                On success
 *       - ESP_ERR_INVALID_STATE: MCP2515 device is not in use
 *       - ESP_ERR_INVALID_ARG:   Invalid argument
 */
esp_err_t canbus_mcp2515_get_mode(const canbus_mcp2515_handle_t handle, mcp2515_mode_t* pMode);

/**
 * @brief Set the operation mode of the MCP2515 device.
 */
esp_err_t canbus_mcp2515_set_mode(canbus_mcp2515_handle_t handle, const mcp2515_mode_t mode, const TickType_t modeChangeDelay);

/**
 * @brief Set the bit timing configuration of the MCP2515 device.
 * @param handle            Handle of the MCP2515 device
 * @param bitTimingConfig   Bit timing configuration
 */
esp_err_t canbus_mcp2515_set_bitrate(canbus_mcp2515_handle_t handle, const mcp2515_bit_timing_config_t* bitTimingConfig);

/**
 * @brief Set MCP2515 receive filter(s).
 * @note Filters apply to both standard and extended frames as follows:
 *        * Standard: mask and filter (16 bits) apply to the Data bytes 01 and 1
 *        * Extended: mask and filter (29 bits) apply to the entire Extended ID field
 * @param handle   Handle of the MCP2515 device
 * @param filter   Filter
 */
esp_err_t canbus_mcp2515_set_receive_filter(canbus_mcp2515_handle_t handle, const mcp2515_receive_filter_t* filter);


esp_err_t canbus_mcp2515_get_transmit_error_count(canbus_mcp2515_handle_t handle, uint8_t* count);
esp_err_t canbus_mcp2515_get_receive_error_count(canbus_mcp2515_handle_t handle, uint8_t* count);





/**
 * @brief Read an MCP2515 register.
 * @param handle          Handle of the MCP2515 device
 * @param mcp2515Register Register to read
 * @param data            Pointer to a memory location which receives the data
 */
esp_err_t mcp2515_read_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, uint8_t* data);

/**
 * @brief Write to an MCP2515 register.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to write
 * @param data                  Data to write
 */
esp_err_t mcp2515_write_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data);

/**
 * @brief Change a set of bits in a MCP2515 register.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to alter
 * @param data                  Data to apply to the register
 * @param mask                 Number of bytes to read
 */
esp_err_t mcp2515_modify_register(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515Register, const uint8_t data, const uint8_t mask);

/**
 * @brief Write to multiple MCP2515 registers in sequence.
 * @param handle                Handle of the MCP2515 device
 * @param mcp2515RegisterStart  Register to start writing
 * @param data                  Data to write
 * @param count                 Number of bytes to write
 */
esp_err_t mcp2515_write_registers(canbus_mcp2515_handle_t handle, const mcp2515_register_t mcp2515RegisterStart, const uint8_t data[], const uint8_t count);


#ifdef __cplusplus
}
#endif