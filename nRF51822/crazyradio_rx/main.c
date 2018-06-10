/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */
#include <nrf.h>

#include <stdio.h>
#include <string.h>

#include "esb.h"
#include "pm.h"
#include "systick.h"


extern void  initialise_monitor_handles(void);

#ifndef SEMIHOSTING
#define printf(...)
#endif

#ifndef DEFAULT_RADIO_RATE
  #define DEFAULT_RADIO_RATE  esbDatarate250K
#endif
#ifndef DEFAULT_RADIO_CHANNEL
  #define DEFAULT_RADIO_CHANNEL 2
#endif

static void mainloop(void);

//static bool boottedFromBootloader;

static void handleRadioCmd(struct esbPacket_s * packet);
static void handleBootloaderCmd(struct esbPacket_s *packet);

int main()
{
  systickInit();
  //memoryInit();

  NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);

#ifdef SEMIHOSTING
  initialise_monitor_handles();
#endif

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  //LED_INIT();

  pmInit();

  if ((NRF_POWER->GPREGRET&0x01) == 0) {
          pmSetState(pmSysRunning);
  }

  //LED_ON();


  esbInit();

  esbSetDatarate(DEFAULT_RADIO_RATE);
  esbSetChannel(DEFAULT_RADIO_CHANNEL);

  mainloop();

  // The main loop should never end
  // TODO see if we should shut-off the system there?
  while(1);

  return 0;
}

void mainloop()
{
  static EsbPacket esbRxPacket;
  bool esbReceived = false;
  //bool slReceived;
  //static int vbatSendTime;
  //static int radioRSSISendTime;
  //static uint8_t rssi;
  //static bool broadcast;

  while(1)
  {

#ifndef CONT_WAVE_TEST

    if ((esbReceived == false) && esbIsRxPacket())
    {
      EsbPacket* packet = esbGetRxPacket();
      //Store RSSI here so that we can send it to STM later
      //rssi = packet->rssi;
      // The received packet was a broadcast, if received on local address 1
      //broadcast = packet->match == ESB_MULTICAST_ADDRESS_MATCH;
      memcpy(esbRxPacket.data, packet->data, packet->size);
      esbRxPacket.size = packet->size;
      esbReceived = true;
      esbReleaseRxPacket(packet);
    }

    if (esbReceived)
    {
      EsbPacket* packet = &esbRxPacket;
      esbReceived = false;

      if((packet->size >= 4) && (packet->data[0]==0xff) && (packet->data[1]==0x03))
      {
        handleRadioCmd(packet);
      }
      else if ((packet->size >2) && (packet->data[0]==0xff) && (packet->data[1]==0xfe))
      {
        handleBootloaderCmd(packet);
      }
    }

#endif

    pmProcess();
  }
}

#define RADIO_CTRL_SET_CHANNEL 1
#define RADIO_CTRL_SET_DATARATE 2
#define RADIO_CTRL_SET_POWER 3

static void handleRadioCmd(struct esbPacket_s *packet)
{
  switch (packet->data[2]) {
    case RADIO_CTRL_SET_CHANNEL:
      esbSetChannel(packet->data[3]);
      break;
    case RADIO_CTRL_SET_DATARATE:
      esbSetDatarate(packet->data[3]);
      break;
    case RADIO_CTRL_SET_POWER:
      esbSetTxPower(packet->data[3]);
      break;
    default:
      break;
  }
}

#define BOOTLOADER_CMD_RESET_INIT 0xFF
#define BOOTLOADER_CMD_RESET      0xF0
#define BOOTLOADER_CMD_ALLOFF     0x01
#define BOOTLOADER_CMD_SYSOFF     0x02
#define BOOTLOADER_CMD_SYSON      0x03
#define BOOTLOADER_CMD_GETVBAT    0x04

static void handleBootloaderCmd(struct esbPacket_s *packet)
{
  static bool resetInit = false;
  static struct esbPacket_s txpk;

  switch (packet->data[2]) {
    case BOOTLOADER_CMD_RESET_INIT:

      resetInit = true;

      txpk.data[0] = 0xff;
      txpk.data[1] = 0xfe;
      txpk.data[2] = BOOTLOADER_CMD_RESET_INIT;

      memcpy(&(txpk.data[3]), (uint32_t*)NRF_FICR->DEVICEADDR, 6);

      txpk.size = 9;
#if BLE
      bleCrazyfliesSendPacket(&txpk);
#endif
      if (esbCanTxPacket()) {
        struct esbPacket_s *pk = esbGetTxPacket();
        memcpy(pk, &txpk, sizeof(struct esbPacket_s));
        esbSendTxPacket(pk);
      }

      break;
    case BOOTLOADER_CMD_RESET:
      if (resetInit && (packet->size == 4)) {
        msDelay(100);
        if (packet->data[3] == 0) {
          NRF_POWER->GPREGRET |= 0x40;
        } else {
          //Set bit 0x20 forces boot to firmware
          NRF_POWER->GPREGRET |= 0x20U;
        }
#ifdef BLE
        sd_nvic_SystemReset();
#else
        NVIC_SystemReset();
#endif
      }
      break;
    case BOOTLOADER_CMD_ALLOFF:
      pmSetState(pmAllOff);
      break;
    case BOOTLOADER_CMD_SYSOFF:
      pmSetState(pmSysOff);
      break;
    case BOOTLOADER_CMD_SYSON:
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
      break;
    case BOOTLOADER_CMD_GETVBAT:
      if (esbCanTxPacket()) {
        float vbat = pmGetVBAT();
        struct esbPacket_s *pk = esbGetTxPacket();

        pk->data[0] = 0xff;
        pk->data[1] = 0xfe;
        pk->data[2] = BOOTLOADER_CMD_GETVBAT;

        memcpy(&(pk->data[3]), &vbat, sizeof(float));
        pk->size = 3 + sizeof(float);

        esbSendTxPacket(pk);
      }
      break;
    default:
      break;
  }
}
