// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include "canbus_mcp2515.h"


#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif