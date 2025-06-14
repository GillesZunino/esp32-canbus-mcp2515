// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once


#include "canbus_mcp2515.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Displays MCP2515 CANINTE register to the log at specified level.
 *
 * @param tag description tag
 * @param caninte CANINTE register value
 * @param level level of the log
 */
#define MCP2515_LOG_CANINTE( tag, caninte, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_caninte_internal(tag, caninte, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 CANINTF register to the log at specified level.
 *
 * @param tag description tag
 * @param canintf CANINTF register value
 * @param level level of the log
 */
#define MCP2515_LOG_CANINTF( tag, canintf, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_canintf_internal(tag, canintf, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 EFLG register to the log at specified level.
 *
 * @param tag description tag
 * @param eflg EFLG register value
 * @param level level of the log
 */
#define MCP2515_LOG_EFLG( tag, eflg, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_eflg_internal(tag, eflg, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 CANSTAT register to the log at specified level.
 *
 * @param tag description tag
 * @param canstat EFLG register value
 * @param level level of the log
 */
#define MCP2515_LOG_CANSTAT( tag, canstat, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_canstat_internal(tag, canstat, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 CANCTRL register to the log at specified level.
 *
 * @param tag description tag
 * @param canctrl CANCTRL register value
 * @param level level of the log
 */
#define MCP2515_LOG_CANCTRL( tag, canctrl, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_canctrl_internal(tag, canctrl, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 TXBnCTRL register to the log at specified level.
 *
 * @param tag description tag
 * @param canctrl TXBnCTRL register value
 * @param level level of the log
 */
#define MCP2515_LOG_TXBnCTRL( tag, txbnctrl, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_txbnctrl_internal(tag, txbnctrl, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 RXB0CTRL register to the log at specified level.
 *
 * @param tag description tag
 * @param rxb0ctrl RXB0CTRL register value
 * @param level level of the log
 */
#define MCP2515_LOG_RXB0CTRL( tag, rxb0ctrl, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_rxb0ctrl_internal(tag, rxb0ctrl, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 RXB1CTRL register to the log at specified level.
 *
 * @param tag description tag
 * @param rxb1ctrl RXB1CTRL register value
 * @param level level of the log
 */
#define MCP2515_LOG_RXB1CTRL( tag, rxb1ctrl, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_rxb1ctrl_internal(tag, rxb1ctrl, level); \
        } \
    } while(0)

/**
 * @brief Displays a CAN V2.0B frame to the log at specified level.
 *
 * @param tag description tag
 * @param frame pointer to can_frame_t structure to log
 * @param level level of the log
 */
#define MCP2515_LOG_CAN_FRAME( tag, frame, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_can_frame_internal(tag, frame, level); \
        } \
    } while(0)

/**
 * @brief Displays the set of receive filters hit for RXB0 to the log at specified level.
 *
 * @param tag description tag
 * @param filtersHit Filters hit bits to log
 * @param level level of the log
 */
#define MCP2515_LOG_RXB0_FILTERS_HIT( tag, filtersHit, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_rxbn_filters_hit_internal(tag, filtersHit, MCP2515_RECEIVE_RXB0, level); \
        } \
    } while(0)

/**
 * @brief Displays the set of receive filters hit for RXB1 to the log at specified level.
 *
 * @param tag description tag
 * @param filtersHit Filters hit bits to log
 * @param level level of the log
 */
#define MCP2515_LOG_RXB1_FILTERS_HIT( tag, filtersHit, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_rxbn_filters_hit_internal(tag, filtersHit, MCP2515_RECEIVE_RXB1, level); \
        } \
    } while(0)



const char* dump_blt_mode(canbus_mcp2515_blt_mode_t mode);
const char* dump_sampling_mode(canbus_mcp2515_sampling_mode_t sampling);
const char* dump_sof_mode(mcp2515_clkout_sof_mode_t mode);
const char* dump_clkout_prescaler(mcp2515_clkout_prescaler_t prescaler);
const char* dump_txnrts_pin_mode(mcp2515_txnrts_pin_mode_t mode);
const char* dump_rxnbf_pin_mode(mcp2515_rxnbf_pin_mode_t mode);
const char* dump_rxnbf_pin(mcp2515_rxnbf_pin_t mode);
const char* dump_wakeup_lowpass_filter(mcp2515_wakeup_lowpass_filter_t mode);
const char* dump_transmit_behavior(mcp2515_transmit_behavior_t mode);
const char* dump_RXFn(mcp2515_RXFn_t rxfn);
const char* dump_TXBn(mcp2515_TXBn_t tbx);
const char* dump_filter_mode(mcp2515_filter_mode_t mode);
const char* dump_mcp2515_mode(mcp2515_mode_t mode);
const char* dump_transmit_priority(mcp2515_transmit_priority_t priority);
const char* dump_receive_buffer(mcp2515_receive_buffer_t buffer);

const char* dump_frame_options(can_frame_options_t options);

void log_caninte_internal(const char* tag, uint8_t caninte, esp_log_level_t log_level);
void log_canintf_internal(const char* tag, uint8_t canintf, esp_log_level_t log_level);
void log_eflg_internal(const char* tag, uint8_t eflg, esp_log_level_t log_level);
void log_canstat_internal(const char* tag, uint8_t canstat, esp_log_level_t log_level);
void log_canctrl_internal(const char* tag, uint8_t canctrl, esp_log_level_t log_level);
void log_txbnctrl_internal(const char* tag, uint8_t txbnctrl, esp_log_level_t log_level);

void log_rxbn_filters_hit_internal(const char* tag, uint8_t filtersHit, mcp2515_receive_buffer_t receiveBuffer, esp_log_level_t log_level);

void log_rxb0ctrl_internal(const char* tag, uint8_t rxb0ctrl, esp_log_level_t log_level);
void log_rxb1ctrl_internal(const char* tag, uint8_t rxb1ctrl, esp_log_level_t log_level);

void log_can_frame_internal(const char* tag, const can_frame_t* frame, esp_log_level_t log_level);


#ifdef __cplusplus
}
#endif