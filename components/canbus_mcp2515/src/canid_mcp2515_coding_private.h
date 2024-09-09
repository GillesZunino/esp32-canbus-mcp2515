// -----------------------------------------------------------------------------------
// Copyright 2024, Gilles Zunino
// -----------------------------------------------------------------------------------

#pragma once


#include <stdbool.h>
#include <stdint.h>


void encode_canid_into_mcp2515_registers_private(uint32_t id, bool isExtendedFrame, uint8_t buffer[4]);
uint32_t decode_canid_from_mcp2515_registers_private(bool isExtendedFrame, const uint8_t buffer[4]);
