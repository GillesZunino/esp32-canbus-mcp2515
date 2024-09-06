// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------


#include <esp_log.h>

#include "canbus_mcp2515_utils.h"


#define CASE_RETURN_STR(const) case const: return #const;


const char* dump_blt_mode(canbus_mcp2515_blt_mode_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_BTL_MODE_EXPLICIT)
        CASE_RETURN_STR(MCP2515_BTL_MODE_GREATER_THAN_PS1_AND_IPT)
    default:
        return "Unknown canbus_mcp2515_blt_mode_t";
    }
}

const char* dump_sampling_mode(canbus_mcp2515_sampling_mode_t sampling) {
    switch (sampling) {
        CASE_RETURN_STR(MCP2515_SAMPLING_MODE_ONCE)
        CASE_RETURN_STR(MCP2515_SAMPLING_MODE_THREE)
    default:
        return "Unknown canbus_mcp2515_sampling_mode_t";
    }
}

// TODO: Multiples values
// const char* dump_interupts(mcp2515_interrupts_t interrupts);

const char* dump_sof_mode(mcp2515_clkout_sof_mode_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_CLKOUT_PIN_OFF)
        CASE_RETURN_STR(MCP2515_CLKOUT_PIN_SOF)
        CASE_RETURN_STR(MCP2515_CLKOUT_PIN_CLKOUT)
    default:
        return "Unknown mcp2515_clkout_sof_mode_t";
    }
}

const char* dump_clkout_prescaler(mcp2515_clkout_prescaler_t prescaler) {
    switch (prescaler) {
        CASE_RETURN_STR(MCP2515_CLKOUT_DIVIDER_1)
        CASE_RETURN_STR(MCP2515_CLKOUT_DIVIDER_2)
        CASE_RETURN_STR(MCP2515_CLKOUT_DIVIDER_4)
        CASE_RETURN_STR(MCP2515_CLKOUT_DIVIDER_8)
    default:
        return "Unknown mcp2515_clkout_prescaler_t";
    }
}

const char* dump_txnrts_pin_mode(mcp2515_txnrts_pin_mode_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_TXnRTS_PIN_REQUEST_TO_SEND)
        CASE_RETURN_STR(MCP2515_TXnRTS_PIN_DIGITAL_INPUT)
    default:
        return "Unknown mcp2515_txnrts_pin_mode_t";
    }
}

const char* dump_rxnbf_pin_mode(mcp2515_rxnbf_pin_mode_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_RXnBF_PIN_DISABLED)
        CASE_RETURN_STR(MCP2515_RXnBF_PIN_BUFFER_FULL_INT)
        CASE_RETURN_STR(MCP2515_RXnBF_PIN_DIGITAL_OUTPUT)
    default:
        return "Unknown mcp2515_rxnbf_pin_mode_t";
    }
}

const char* dump_rxnbf_pin(mcp2515_rxnbf_pin_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_RXnBF_PIN_RX0)
        CASE_RETURN_STR(MCP2515_RXnBF_PIN_RX1)
    default:
        return "Unknown mcp2515_rxnbf_pin_t";
    }
}

const char* dump_wakeup_lowpass_filter(mcp2515_wakeup_lowpass_filter_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_WAKEUP_LOWPASS_FILTER_DISABLED)
        CASE_RETURN_STR(MCP2515_WAKEUP_LOWPASS_FILTER_ENABLED)
    default:
        return "Unknown mcp2515_wakeup_lowpass_filter_t";
    }
}

const char* dump_transmit_behavior(mcp2515_transmit_behavior_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_TRANSMIT_BEHAVIOR_NORMAL)
        CASE_RETURN_STR(MCP2515_TRANSMIT_BEHAVIOR_ABORT)
    default:
        return "Unknown mcp2515_transmit_behavior_t";
    }
}

const char* dump_RXFn(mcp2515_RXFn_t rxfn) {
    switch (rxfn) {
        CASE_RETURN_STR(RXF0)
        CASE_RETURN_STR(RXF1)
        CASE_RETURN_STR(RXF2)
        CASE_RETURN_STR(RXF3)
        CASE_RETURN_STR(RXF4)
        CASE_RETURN_STR(RXF5)
    default:
        return "Unknown mcp2515_RXFn_t";
    }
}

const char* dump_TXBn(mcp2515_TXBn_t tbx) {
    switch (tbx) {
        CASE_RETURN_STR(MCP2515_TXB_NONE)
        CASE_RETURN_STR(MCP2515_TXB_AUTO)
        CASE_RETURN_STR(MCP2515_TXB0)
        CASE_RETURN_STR(MCP2515_TXB1)
        CASE_RETURN_STR(MCP2515_TXB2)
    default:
        return "Unknown mcp2515_TXBn_t";
    }
}

const char* dump_filter_mode(mcp2515_filter_mode_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_FILTER_STANDARD_FRAME)
        CASE_RETURN_STR(MCP2515_FILTER_EXTENDED_FRAME)
    default:
        return "Unknown mcp2515_filter_mode_t";
    }
}

const char* dump_mcp2515_mode(mcp2515_mode_t mode) {
    switch (mode) {
        CASE_RETURN_STR(MCP2515_MODE_NORMAL)
        CASE_RETURN_STR(MCP2515_MODE_SLEEP)
        CASE_RETURN_STR(MCP2515_MODE_LOOPBACK)
        CASE_RETURN_STR(MCP2515_MODE_LISTENONLY)
        CASE_RETURN_STR(MCP2515_MODE_CONFIG)
    default:
        return "Unknown mcp2515_mode_t";
    }
}

const char* dump_transmit_priority(mcp2515_transmit_priority_t priority) {
    switch (priority) {
        CASE_RETURN_STR(MCP1515_TRANSMIT_PRIORITY_LOW)
        CASE_RETURN_STR(MCP1515_TRANSMIT_PRIORITY_MEDIUM)
        CASE_RETURN_STR(MCP1515_TRANSMIT_PRIORITY_HIGH)
        CASE_RETURN_STR(MCP1515_TRANSMIT_PRIORITY_HIGHEST)
    default:
        return "Unknown mcp2515_transmit_priority_t";
    }
}

const char* dump_receive_buffer(mcp2515_receive_buffer_t buffer) {
    switch (buffer) {
        CASE_RETURN_STR(MCP2515_RECEIVE_RXB0)
        CASE_RETURN_STR(MCP2515_RECEIVE_RXB1)
    default:
        return "Unknown mcp2515_receive_buffer_t";
    }
}

const char* dump_frame_options(can_frame_options_t options) {
    switch (options) {
        CASE_RETURN_STR(CAN_FRAME_OPTION_RTR)
        CASE_RETURN_STR(CAN_FRAME_OPTION_EXTENDED)
    default:
        return "Unknown can_frame_options_t";
    }
}

void log_canintf_internal(const char *tag, uint8_t canintf, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|                 CANINTF: 0x%02X                 |", canintf);
    ESP_LOG_LEVEL(log_level, tag, "|MERRF|WAKIF|ERRIF|TX2IF|TX1IF|TX0IF|RX1IF|RX0IF|");
    ESP_LOG_LEVEL(log_level, tag, "|  %s  |  %s  |  %s  |  %s  |  %s  |  %s  |  %s  |  %s  |",
        canintf & (1 << 7) ? "x" : " ",
        canintf & (1 << 6) ? "x" : " ",
        canintf & (1 << 5) ? "x" : " ",
        canintf & (1 << 4) ? "x" : " ",
        canintf & (1 << 3) ? "x" : " ",
        canintf & (1 << 2) ? "x" : " ",
        canintf & (1 << 1) ? "x" : " ",
        canintf & (1 << 0) ? "x" : " ");
}

void log_eflg_internal(const char *tag, uint8_t eflg, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|                   EFLG: 0x%02X                   |", eflg);
    ESP_LOG_LEVEL(log_level, tag, "|RX1OVR|RX0OVR|TXBOFF|TXEP|RXEP|TXWAR|RXWAR|EWARN|");
    ESP_LOG_LEVEL(log_level, tag, "|   %s  |   %s  |   %s |  %s |  %s  |  %s  |  %s  |  %s  |",
        eflg & (1 << 7) ? "x" : " ",
        eflg & (1 << 6) ? "x" : " ",
        eflg & (1 << 5) ? "x" : " ",
        eflg & (1 << 4) ? "x" : " ",
        eflg & (1 << 3) ? "x" : " ",
        eflg & (1 << 2) ? "x" : " ",
        eflg & (1 << 1) ? "x" : " ",
        eflg & (1 << 0) ? "x" : " ");
}

void log_canstat_internal(const char *tag, uint8_t canstat, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|                  CANSTAT: 0x%02X                 |", canstat);
    ESP_LOG_LEVEL(log_level, tag, "|OPMOD2|OPMOD1|OPMOD0| -- |ICOD2|ICOD1|ICOD0| -- |");
    ESP_LOG_LEVEL(log_level, tag, "|   %s  |   %s  |   %s  | -- |  %s  |  %s  |  %s  | -- |",
        canstat & (1 << 7) ? "x" : " ",
        canstat & (1 << 6) ? "x" : " ",
        canstat & (1 << 5) ? "x" : " ",
        canstat & (1 << 3) ? "x" : " ",
        canstat & (1 << 2) ? "x" : " ",
        canstat & (1 << 1) ? "x" : " ");
}

void log_canctrl_internal(const char *tag, uint8_t canctrl, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|                   CANCTRL: 0x%02X                   |", canctrl);
    ESP_LOG_LEVEL(log_level, tag, "|REQOP2|REQOP1|REQOP0|ABAT|OSM|CLKEN|CLKPRE1|CLKPRE0|");
    ESP_LOG_LEVEL(log_level, tag, "|   %s  |   %s  |   %s  |  %s | %s |  %s  |   %s   |   %s   |",
        canctrl & (1 << 7) ? "x" : " ",
        canctrl & (1 << 6) ? "x" : " ",
        canctrl & (1 << 5) ? "x" : " ",
        canctrl & (1 << 4) ? "x" : " ",
        canctrl & (1 << 3) ? "x" : " ",
        canctrl & (1 << 2) ? "x" : " ",
        canctrl & (1 << 1) ? "x" : " ",
        canctrl & (1 << 0) ? "x" : " ");
}