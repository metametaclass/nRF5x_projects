//#ifndef RADIO_TX_H
//#define RADIO_TX_H
//#endif

#pragma once

#include "bit_swap.h"


#define MAX_RADIO_PAYLOAD_SIZE (32UL)

void radio_initialization();


int send_packet(uint8_t *packet);
