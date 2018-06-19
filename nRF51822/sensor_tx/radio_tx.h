//#ifndef RADIO_TX_H
//#define RADIO_TX_H
//#endif

#pragma once

#include "bit_swap.h"

void radio_initialization();


int32_t send_packet(uint8_t *packet);
