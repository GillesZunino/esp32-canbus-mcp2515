// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "can.h"


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
    mcp2515_spi_config_t spi_cfg;       ///< SPi configuration for MCp2515
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
 * @brief MCP2515 Interrupts.
 */
typedef enum {
    MCP2515_INTERRUPT_DISABLED = 0,
    MCP2515_INTERRUPT_RX0_MSG_RECEIVED = 0x01,
    MCP2515_INTERRUPT_RX1_MSG_RECEIVED = 0x02,
    MCP2515_INTERRUPT_TX0_MSG_SENT = 0x04,
    MCP2515_INTERRUPT_TX1_MSG_SENT = 0x08,
    MCP2515_INTERRUPT_TX2_MSG_SENT = 0x10,
    MCP2515_INTERRUPT_ERROR = 32,
    MCP2515_INTERRUPT_WAKEUP = 64,
    MCP2515_INTERRUPT_MESSAGE_ERROR = 128,

    MCP2515_INTERRUPT_ALL_MSG_RECEIVED = MCP2515_INTERRUPT_RX0_MSG_RECEIVED | MCP2515_INTERRUPT_RX1_MSG_RECEIVED,
    MCP2515_INTERRUPT_ALL_MSG_SENT = MCP2515_INTERRUPT_TX0_MSG_SENT | MCP2515_INTERRUPT_TX1_MSG_SENT | MCP2515_INTERRUPT_TX2_MSG_SENT,

    MCP2515_INTERRUPT_ALL = MCP2515_INTERRUPT_ALL_MSG_RECEIVED | MCP2515_INTERRUPT_ALL_MSG_SENT | MCP2515_INTERRUPT_ERROR | MCP2515_INTERRUPT_WAKEUP | MCP2515_INTERRUPT_MESSAGE_ERROR
} mcp2515_interrupts_t;

/**
 * @brief MCP2515 interrupt handler.
 */
typedef void (*mcp2515_interrupt_handler_t)(mcp2515_interrupts_t interrupt);

/**
 * @brief MCP2515 Interrupt configuration.
 */
typedef struct mcp2515_interrupt_config {
    gpio_num_t intr_io_num;
    mcp2515_interrupts_t flags;
    mcp2515_interrupt_handler_t handler;
} mcp2515_interrupt_config_t;

/**
 * @brief MCP2515 CLKOUT / SOF mode.
 */
typedef enum {
    MCP2515_CLKOUT_PIN_OFF = 0,     ///< CLKOUT/SOF pin disabled
    MCP2515_CLKOUT_PIN_SOF = 1,     ///< CLKOUT/SOF pin as SOF signal
    MCP2515_CLKOUT_PIN_CLKOUT = 2   ///< CLKOUT/SOF pin as CLKOUT signal
} mcp2515_clkout_sof_mode_t;

/**
 * @brief MCP2515 CLKOUT prescaler configuration.
 */
typedef enum {
    MCP2515_CLKOUT_DIVIDER_1 = 0,    ///< CLKOUT = fosc
    MCP2515_CLKOUT_DIVIDER_2 = 1,    ///< CLKOUT = fosc / 2
    MCP2515_CLKOUT_DIVIDER_4 = 2,    ///< CLKOUT = fosc / 4
    MCP2515_CLKOUT_DIVIDER_8 = 3,    ///< CLKOUT = fosc / 8
} mcp2515_clkout_prescaler_t;

/**
 * @brief MCP2515 CLKOUT pin configuration.
 */
typedef struct mcp2515_clkout_sof_config {
    mcp2515_clkout_sof_mode_t mode;          ///< CLKOUT/SOF pin mode
    mcp2515_clkout_prescaler_t prescaler;    ///< CLKOUT prescaler when CLKOUT/SOF pin is confgiured for CLKOUT
} mcp2515_clkout_sof_config_t;


/**
 * @brief MCP2515 TXnRTS pin mode.
 */
typedef enum {
    MCP2515_TXnRTS_PIN_REQUEST_TO_SEND = 0,    ///< TXnRTS pins as Request-to-Send
    MCP2515_TXnRTS_PIN_DIGITAL_INPUT = 1       ///< TXnRTS pins as digital input
} mcp2515_txnrts_pin_mode_t;

typedef struct mcp2515_txnrts_pins_config {
    mcp2515_txnrts_pin_mode_t tx0rts_mode;
    mcp2515_txnrts_pin_mode_t tx1rts_mode;
    mcp2515_txnrts_pin_mode_t tx2rts_mode;
} mcp2515_txnrts_pins_config_t;

#define MCP2515_TXnRTS_PIN_TX0 1
#define MCP2515_TXnRTS_PIN_TX1 2
#define MCP2515_TXnRTS_PIN_TX2 4



typedef enum {
    MCP2515_RXnBF_PIN_DISABLED = 0,
    MCP2515_RXnBF_PIN_BUFFER_FULL_INT = 1,
    MCP2515_RXnBF_PIN_DIGITAL_OUTPUT = 2
} mcp2515_rxnbf_pin_mode_t;

typedef struct mcp2515_rxnbf_pins_config {
    mcp2515_rxnbf_pin_mode_t rx0bf;
    mcp2515_rxnbf_pin_mode_t rx1bf;
} mcp2515_rxnbf_pins_config_t;

typedef enum {
    MCP2515_RXnBF_PIN_RX0 = 1,
    MCP2515_RXnBF_PIN_RX1 = 2
} mcp2515_rxnbf_pin_t;

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
 * @brief MCP2515 Transmit registers.
 */
typedef enum  {
    MCP2515_TXB_AUTO = -1,  ///< Automatically select the first available register
    MCP2515_TXB0 = 0,   ///< Use TXB0
    MCP2515_TXB1 = 1,   ///< Use TXB1
    MCP2515_TXB2 = 2    ///< Use TXB2
} mcp2515_TXBn_t;


/**
 * @brief Trasnmit options for sending CAN frames.
 */
typedef struct canbus_mcp2515_transmit_options {
    mcp2515_TXBn_t txb;    ///< Transmit register to use
} canbus_mcp2515_transmit_options_t;

/**
 * @brief MCP2515 error flags for EFLG interpretation.
 */
#define MCP2515_EFLAG_RX1OVR 0x80       ///< Receive buffer 1 overflow flag. Set when a valid message is received in RX1and an interupt is pending
#define MCP2515_EFLAG_RX0OVR 0x40       ///< Receive buffer 0 overflow flag. Set when a valid message is received in RX0 and an interupt is pending
#define MCP2515_EFLAG_TXBO 0x20         ///< Trasnmit Bus-Off flag. Set when TEC is greater or equal than 255
#define MCP2515_EFLAG_TEXP 0x10         ///< Transmit Error-Passive flag. Set when TEC is greater or equal than 128
#define MCP2515_EFLAG_REXP 0x08         ///< Receive Error-Passive flag. Set when REC is greater or equal than 128
#define MCP2515_EFLAG_TXWARN 0x04       ///< Transmit error warning flag. Set when TEC is greater or equal than 96
#define MCP2515_EFLAG_RXWARN 0x02       ///< Receive error warning flag. Set when REC is greater or equal than 96
#define MCP2515_EFLAG_EWARN 0x01        ///< Error warning flag. Set when REC or TEC is greater or equal than 96
 

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



esp_err_t canbus_mcp2515_configure_interrupts(canbus_mcp2515_handle_t handle, const mcp2515_interrupt_config_t* config);
esp_err_t canbus_mcp2515_get_interrupt_flags(canbus_mcp2515_handle_t handle, uint8_t* flags);
esp_err_t canbus_mcp2515_reset_interrupt_flags(canbus_mcp2515_handle_t handle, mcp2515_interrupts_t flags);


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
 * @brief COnfigure MCP2515 one-shot mode.
 * @param handle    Handle of the MCP2515 device
 * @param enable    true to enable One-Shot mode, false to disable.
 */
esp_err_t canbus_mcp2515_set_oneshot_mode(canbus_mcp2515_handle_t handle, bool enable);

/**
 * @brief Set the bit timing configuration of the MCP2515 device.
 * @param handle            Handle of the MCP2515 device
 * @param bitTimingConfig   Bit timing configuration
 */
esp_err_t canbus_mcp2515_configure_bitrate(canbus_mcp2515_handle_t handle, const mcp2515_bit_timing_config_t* bitTimingConfig);

/**
 * @brief Set MCP2515 receive filter(s).
 * @note Filters apply to both standard and extended frames as follows:
 *        * Standard: mask and filter (16 bits) apply to the Data bytes 01 and 1
 *        * Extended: mask and filter (29 bits) apply to the entire Extended ID field
 * @param handle   Handle of the MCP2515 device
 * @param filter   Filter
 */
esp_err_t canbus_mcp2515_configure_receive_filter(canbus_mcp2515_handle_t handle, const mcp2515_receive_filter_t* filter);

/**
 * @brief Configure MCP2515 CLKOUT pin behavior.
 * @param handle    Handle of the MCP2515 device
 * @param config    CLKOUT/SOF pin configuration
 */
esp_err_t canbus_mcp2515_configure_clkout_sof(canbus_mcp2515_handle_t handle, const mcp2515_clkout_sof_config_t* config);

/**
 * @brief Configure MCP2515 TXnRST pins behavior.
 * @param handle    Handle of the MCP2515 device
 * @param config   TXnRTS pins mode
 */
esp_err_t canbus_mcp2515_configure_txnrts(canbus_mcp2515_handle_t handle, const mcp2515_txnrts_pins_config_t* config);

/**
 * @brief Read state of MCP2515 TXnRTS pins as digital inputs.
 * @note For RXnRTS pins to act as difitial input, configure the MCP2515 by calling canbus_mcp2515_set_txnrts().
 * @param handle    Handle of the MCP2515 device
 * @param txrts     Pointer to a memory location which receives the state of the TXnRTS pins
 */
esp_err_t canbus_mcp2515_get_txnrts(canbus_mcp2515_handle_t handle, uint8_t* txrts);

/**
 * @brief Transmit an MCP2515 register.
 * @param handle    Handle of the MCP2515 device
 * @param frame     Frame to transmit
 * @param options   Transmit options
 */
esp_err_t canbus_mcp2515_transmit(canbus_mcp2515_handle_t handle, const can_frame_t* frame, const canbus_mcp2515_transmit_options_t* options); 


esp_err_t canbus_mcp2515_configure_rxnbf(canbus_mcp2515_handle_t handle,const mcp2515_rxnbf_pins_config_t* config);
esp_err_t canbus_mcp2515_set_rxnbf(canbus_mcp2515_handle_t handle, mcp2515_rxnbf_pin_t rxnbf, bool level);


esp_err_t canbus_mcp2515_get_transmit_error_count(canbus_mcp2515_handle_t handle, uint8_t* count);
esp_err_t canbus_mcp2515_get_receive_error_count(canbus_mcp2515_handle_t handle, uint8_t* count);

esp_err_t canbus_mcp1515_get_error_flags(canbus_mcp2515_handle_t handle, uint8_t* flags);


#ifdef __cplusplus
}
#endif