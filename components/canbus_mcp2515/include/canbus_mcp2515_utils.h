// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include "canbus_mcp2515.h"


#ifdef __cplusplus
extern "C" {
#endif


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
            log_canintf_internal( tag, canintf.flags, level); \
        } \
    } while(0)

/**
 * @brief Displays MCP2515 EFLG register to the log at specified level.
 *
 * @param tag description tag
 * @param canintf EFLG register value
 * @param level level of the log
 */
#define MCP2515_LOG_EFLG( tag, eflg, level ) \
    do { \
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            log_eflg_internal( tag, eflg, level); \
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

void log_canintf_internal(const char *tag, uint8_t canintf, esp_log_level_t log_level);
void log_eflg_internal(const char *tag, uint8_t eflg, esp_log_level_t log_level);


#ifdef __cplusplus
}
#endif