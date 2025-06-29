// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------


#include <esp_log.h>

#include "canbus_mcp2515_log_utils.h"


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

void log_caninte_internal(const char* tag, uint8_t caninte, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|                 CANINTE: 0x%02X                 |", caninte);
    ESP_LOG_LEVEL(log_level, tag, "|MERRE|WAKIE|ERRIE|TX2IE|TX1IE|TX0IE|RX1IE|RX0IE|");
    ESP_LOG_LEVEL(log_level, tag, "|  %s  |  %s  |  %s  |  %s  |  %s  |  %s  |  %s  |  %s  |",
        caninte & (1 << 7) ? "x" : " ",
        caninte & (1 << 6) ? "x" : " ",
        caninte & (1 << 5) ? "x" : " ",
        caninte & (1 << 4) ? "x" : " ",
        caninte & (1 << 3) ? "x" : " ",
        caninte & (1 << 2) ? "x" : " ",
        caninte & (1 << 1) ? "x" : " ",
        caninte & (1 << 0) ? "x" : " ");
}

void log_canintf_internal(const char* tag, uint8_t canintf, esp_log_level_t log_level) {
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

void log_eflg_internal(const char* tag, uint8_t eflg, esp_log_level_t log_level) {
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

void log_canstat_internal(const char* tag, uint8_t canstat, esp_log_level_t log_level) {
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

void log_canctrl_internal(const char* tag, uint8_t canctrl, esp_log_level_t log_level) {
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

void log_txbnctrl_internal(const char* tag, uint8_t txbnctrl, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|             TXBnCTRL: 0x%02X              |", txbnctrl);
    ESP_LOG_LEVEL(log_level, tag, "| -- |ABTF|MLOA|TXERR|TXREQ| -- |TXP1|TXP0|");
    ESP_LOG_LEVEL(log_level, tag, "| -- | %s  | %s  |  %s  |  %s  | -- | %s  | %s  |",
        txbnctrl & (1 << 6) ? "x" : " ",
        txbnctrl & (1 << 5) ? "x" : " ",
        txbnctrl & (1 << 4) ? "x" : " ",
        txbnctrl & (1 << 3) ? "x" : " ",
        txbnctrl & (1 << 1) ? "x" : " ",
        txbnctrl & (1 << 0) ? "x" : " ");
}

void log_rxbn_filters_hit_internal(const char* tag, uint8_t filtersHit, mcp2515_receive_buffer_t receiveBuffer, esp_log_level_t log_level) {
    mcp2515_receive_filter_hit_t filtersHitUnion =  { .flags = filtersHit };
    ESP_LOG_LEVEL(log_level, tag, "|          FLT: 0x%02X          |", filtersHit);
    switch (receiveBuffer) {
        case MCP2515_RECEIVE_RXB0:
            ESP_LOG_LEVEL(log_level, tag, "|  RXB0   |        RXB1       |");
            ESP_LOG_LEVEL(log_level, tag, "|RXF0|RXF1|                   |");
            ESP_LOG_LEVEL(log_level, tag, "| %s  | %s  |                   |",
                filtersHitUnion.filters.rxf0 ? "x" : " ",
                filtersHitUnion.filters.rxf1 ? "x" : " ");
            break;

        // In rollover mode, RXB1 can indicate a filter hit of RXF0 or RXF1 if the message was rolled over
        case MCP2515_RECEIVE_RXB1:
            ESP_LOG_LEVEL(log_level, tag, "|  RXB0   |        RXB1       |");
            ESP_LOG_LEVEL(log_level, tag, "|RXF0|RXF1|RXF2|RXF3|RXF4|RXF5|");
            ESP_LOG_LEVEL(log_level, tag, "| %s  | %s  | %s  | %s  | %s  | %s  |",
                filtersHitUnion.filters.rxf0 ? "x" : " ",
                filtersHitUnion.filters.rxf1 ? "x" : " ",
                filtersHitUnion.filters.rxf2 ? "x" : " ",
                filtersHitUnion.filters.rxf3 ? "x" : " ",
                filtersHitUnion.filters.rxf4 ? "x" : " ",
                filtersHitUnion.filters.rxf5 ? "x" : " ");
            break;

        default:
            ESP_LOG_LEVEL(log_level, tag, "|       UNKNOWN REGISTER      |");
    }
}

void log_rxb0ctrl_internal(const char* tag, uint8_t rxb0ctrl, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|               RXB0CTRL: 0x%02X               |", rxb0ctrl);
    ESP_LOG_LEVEL(log_level, tag, "| -- |RXM1|RXM0| -- |RXRTR|BUKT|BUKT1|CLKPRE0|");
    ESP_LOG_LEVEL(log_level, tag, "| -- | %s  | %s  | -- |  %s  | %s  |  %s  |   %s   |",
        rxb0ctrl & (1 << 6) ? "x" : " ",
        rxb0ctrl & (1 << 5) ? "x" : " ",
        rxb0ctrl & (1 << 3) ? "x" : " ",
        rxb0ctrl & (1 << 2) ? "x" : " ",
        rxb0ctrl & (1 << 1) ? "x" : " ",
        rxb0ctrl & (1 << 0) ? "x" : " ");
}

void log_rxb1ctrl_internal(const char* tag, uint8_t rxb1ctrl, esp_log_level_t log_level) {
    ESP_LOG_LEVEL(log_level, tag, "|                 RXB0CTRL: 0x%02X                  |", rxb1ctrl);
    ESP_LOG_LEVEL(log_level, tag, "| -- |RXM1|RXM0| -- |RXRTR|FILHIT2|FILHIT1|FILHIT0|");
    ESP_LOG_LEVEL(log_level, tag, "| -- | %s  | %s  | -- |  %s  |   %s   |   %s   |   %s   |",
        rxb1ctrl & (1 << 6) ? "x" : " ",
        rxb1ctrl & (1 << 5) ? "x" : " ",
        rxb1ctrl & (1 << 3) ? "x" : " ",
        rxb1ctrl & (1 << 2) ? "x" : " ",
        rxb1ctrl & (1 << 1) ? "x" : " ",
        rxb1ctrl & (1 << 0) ? "x" : " ");
}

void log_can_frame_internal(const char* tag, const can_frame_t* frame, esp_log_level_t log_level) {
    char id[8 + 1 + 1];
    if (frame->options & CAN_FRAME_OPTION_EXTENDED) {
        snprintf(id, sizeof(id) / sizeof(id[0]), "%08lX", frame->id);
    } else {
        snprintf(id, sizeof(id) / sizeof(id[0]), "%03X     ", (uint16_t) frame->id);
    }
    ESP_LOG_LEVEL(log_level, tag, "|ID:0x%s| %s  | %s  |DLC: %1d|                    |",
        id, frame->options & CAN_FRAME_OPTION_EXTENDED ? "IDE" : "   ", frame->options & CAN_FRAME_OPTION_RTR ? "RTR" : "   ",
        frame->dlc);
    
    char dl0[4 + 1] = "    ";
    if (frame->dlc >= 1) { snprintf(dl0, sizeof(dl0) / sizeof(dl0[0]), "0x%02X", frame->data[0]); }

    char dl1[4 + 1] = "    ";
    if (frame->dlc >= 2) { snprintf(dl1, sizeof(dl1) / sizeof(dl1[0]), "0x%02X", frame->data[1]); }

    char dl2[4 + 1] = "    ";
    if (frame->dlc >= 3) { snprintf(dl2, sizeof(dl2) / sizeof(dl2[0]), "0x%02X", frame->data[2]); }

    char dl3[4 + 1] = "    ";
    if (frame->dlc >= 4) { snprintf(dl3, sizeof(dl3) / sizeof(dl3[0]), "0x%02X", frame->data[3]); }

    char dl4[4 + 1] = "    ";
    if (frame->dlc >= 5) { snprintf(dl4, sizeof(dl4) / sizeof(dl4[0]), "0x%02X", frame->data[4]); }

    char dl5[4 + 1] = "    ";
    if (frame->dlc >= 6) { snprintf(dl5, sizeof(dl5) / sizeof(dl5[0]), "0x%02X", frame->data[5]); }

    char dl6[4 + 1] = "    ";
    if (frame->dlc >= 7) { snprintf(dl6, sizeof(dl6) / sizeof(dl6[0]), "0x%02X", frame->data[6]); }

    char dl7[4 + 1] = "    ";
    if (frame->dlc == 8) { snprintf(dl7, sizeof(dl7) / sizeof(dl7[0]), "0x%02X", frame->data[7]); }

    ESP_LOG_LEVEL(log_level, tag, "| %s | %s | %s | %s | %s | %s | %s | %s |", dl0, dl1, dl2, dl3, dl4, dl5, dl6, dl7);
}