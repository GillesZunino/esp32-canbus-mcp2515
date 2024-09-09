// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#include <esp_attr.h>

#include "canid_mcp2515_coding_private.h"


#define CAN_SFF_MASK 0x000007FFUL ///< Standard frame format (SFF) - Max ID mask
#define CAN_EFF_MASK 0x1FFFFFFFUL ///< Extended frame format (EFF) - Max ID mask


FORCE_INLINE_ATTR uint8_t internal_get_sid10_to_sid3_from_standard_id(const uint32_t id) {
    return ((id & CAN_SFF_MASK) >> 3) & 0xFF;
}

FORCE_INLINE_ATTR uint8_t internal_get_sid10_to_sid3_from_extended_id(const uint32_t id) {
    return ((id &0x1F000000) >> 21) | ((id & 0xE00000) >> 21);
}

FORCE_INLINE_ATTR uint8_t internal_get_sid2_to_sid0_from_standard_id(const uint32_t id) {
    return (id & CAN_SFF_MASK) & 0x07;
}

FORCE_INLINE_ATTR uint8_t internal_get_sid2_to_sid0_from_extended_id(const uint32_t id) {
    return (id & 0x1C0000) >> 18;
}

FORCE_INLINE_ATTR uint32_t internal_get_eid17_to_eid16(const uint32_t id) {
    return ((id & CAN_EFF_MASK) >> 16) & 0x03;
}

FORCE_INLINE_ATTR uint32_t internal_get_eid15_to_eid8(const uint32_t id) {
    return ((id & CAN_EFF_MASK) >> 8) & 0xFF;
}

FORCE_INLINE_ATTR uint32_t internal_get_eid7_to_eid0(const uint32_t id) {
    return (id & CAN_EFF_MASK) & 0xFF;
}


void encode_canid_into_mcp2515_registers_private(uint32_t id, bool isExtendedFrame, uint8_t buffer[4]) {
    // Prepare Standard ID - TXBnSIDH and TXBnSIDL
    buffer[0] = isExtendedFrame ? internal_get_sid10_to_sid3_from_extended_id(id) : internal_get_sid10_to_sid3_from_standard_id(id);
    buffer[1] = isExtendedFrame ? (internal_get_sid2_to_sid0_from_extended_id(id) << 5) | (0x08 | internal_get_eid17_to_eid16(id)) : (internal_get_sid2_to_sid0_from_standard_id(id) << 5);

    // Prepare Extended ID - TXBnEID8 and TXBnEID0
    if (isExtendedFrame) {
        buffer[2] = internal_get_eid15_to_eid8(id); 
        buffer[3] = internal_get_eid7_to_eid0(id);
    }
}

uint32_t decode_canid_from_mcp2515_registers_private(bool isExtendedFrame, const uint8_t buffer[4]) {
    uint32_t canId;

    if (isExtendedFrame) {
        // Extended frame ID from RXBnSIDH RXBnSIDL
        canId = (((uint32_t) buffer[0]) << 21) | 
                    (((((uint32_t) buffer[1]) & 0xE0) >> 5) << 18) | ((((uint32_t) buffer[1]) & 0x03) << 16) | 
                    //               RXBnEID8                                 RXBnEID0
                    (((uint32_t) buffer[2]) << 8) | ((uint32_t) buffer[3]);
    } else {
        // Standard frame ID from RXBnSIDH RXBnSIDL
        canId = (((uint32_t) buffer[0]) << 3) | ((((uint32_t) buffer[1]) & 0xE0) >> 5);
    }

    return canId;
}