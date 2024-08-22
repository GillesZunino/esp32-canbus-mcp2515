// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Special address description flags for the CAN_ID.
 */
#define CAN_EFF_FLAG 0x80000000UL ///< EFF/SFF is set in the MSB
#define CAN_RTR_FLAG 0x40000000UL ///< Remote transmission request
#define CAN_ERR_FLAG 0x20000000UL ///< Error message frame

/**
 * @brief Valid bits in CAN ID for frame formats.
 */
#define CAN_SFF_MASK 0x000007FFUL ///< Standard frame format (SFF)
#define CAN_EFF_MASK 0x1FFFFFFFUL ///< Extended frame format (EFF)
#define CAN_ERR_MASK 0x1FFFFFFFUL ///< Omit EFF, RTR, ERR flags

/** 
 * @brief Controller Area Network Identifier structure.
 * @note bit 0-28 : CAN identifier (11/29 bit)
 *       bit 29   : Error message frame flag (0 = data frame, 1 = error message)
 *       bit 30   : Remote transmission request flag (1 = rtr frame)
 *       bit 31   : Frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */


#define CAN_SFF_ID_BITS     11 ///< Standard CAN ID occupies 11 bits
#define CAN_EFF_ID_BITS     29 ///< Extended CAN ID occupies 29 bits

/**
 * @brief CAN payload length and Data Length Code (DLC) definitions according to ISO 11898-1.
 */
#define CAN_MAX_DLC 8  ///< Maximum number of data bytes in a CAN data frame
#define CAN_MAX_DLEN 8 ///< CAN frame payload maximum length in byte
typedef enum {
    CAN_FRAME_OPTION_RTR = 1,                ///< Frame is a Remote Transmission Request
    CAN_FRAME_OPTION_EXTENDED = 2            ///< Frame is an Extended Frame
} can_frame_options_t;

/**
 * @brief CAN frame structure (CAN 2.0B).
 */
typedef struct can_frame {
    can_frame_options_t options;                            ///< Extended Frame, RTR ... (Logical OR of can_frame_options_t flags)
    uint32_t id;                                            ///< Standard (11 bits) or extended (29 bits) CAN ID
    uint8_t dlc;                                            ///< Frame payload length in byte (0 .. CAN_MAX_DLEN)
    uint8_t data[CAN_MAX_DLC] __attribute__((aligned(8)));  ///< Payload data
}  __attribute__((packed)) can_frame_t;


#ifdef __cplusplus
}
#endif