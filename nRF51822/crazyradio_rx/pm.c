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
#include <stdbool.h>

#include <nrf.h>
#include <nrf_gpio.h>

#include "pm.h"
//#include "button.h"
//#include "led.h"
#include "systick.h"
//#include "uart.h"

#define HAS_TI_CHARGER
#define HAS_BAT_SINK


#define AIN_VBAT ADC_CONFIG_PSEL_AnalogInput2
#define AIN_ISET ADC_CONFIG_PSEL_AnalogInput4
#define AIN_VBAT_DIVIDER ADC_CONFIG_INPSEL_AnalogInputTwoThirdsPrescaling
#define ADC_SCALER 3
#define ADC_DIVIDER (3.0/2.0)


static PmState state;
static PmState targetState;
static bool systemBootloader=false;

typedef enum {adcVBAT, adcISET} ADCState;

static ADCState adcState = adcVBAT;

static float vBat;
static float iSet;

void pmInit()
{
  state = pmSysOff; //When NRF starts, the system is OFF
  targetState = state;


}

bool pmUSBPower(void) {
  return 0;
}

bool pmIsCharging(void) {
  return 0;
}

static void pmStartAdc(ADCState state)
{
    if (state == adcVBAT) {
        NRF_ADC->CONFIG = AIN_VBAT << ADC_CONFIG_PSEL_Pos |
            ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos |
            ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos |
            AIN_VBAT_DIVIDER << ADC_CONFIG_INPSEL_Pos;
    }
    if (state == adcISET) {
        NRF_ADC->CONFIG = AIN_ISET << ADC_CONFIG_PSEL_Pos |
            ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos |
            ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos |
            ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos;

    }
    adcState = state;
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->TASKS_START = 0x01;
}

static void pmNrfPower(bool enable)
{
  if (!enable) {
    //stop NRF
    //LED_OFF();

    NRF_POWER->GPREGRET |= 0x01; // Workaround for not being able to determine reset reason...
    NRF_POWER->SYSTEMOFF = 1UL;

    while(1);
  }
}

static void pmDummy(bool enable) {
  ;
}

static void pmPowerSystem(bool enable)
{
}

static void pmSysBoot(bool enable)
{
}

static void pmRunSystem(bool enable)
{
  if (enable) {
    pmStartAdc(adcVBAT);
  } else {
  }
}

/* User API to set and get power state */
void pmSetState(PmState newState)
{
  targetState = newState;
}

PmState pmGetState()
{
  return state;
}

float pmGetVBAT(void) {
    return vBat;
}

float pmGetISET(void) {
  return iSet;
}

void pmSysBootloader(bool enable)
{
  systemBootloader = enable;
}

/* Defines all the power states for easy-usage by a generic state machine */
const struct {
  void (*call)(bool enable);
} statesFunctions[] = {
  [pmAllOff] =       { .call = pmDummy },
  [pmSysOff] =       { .call = pmNrfPower },
  [pmSysPowered] =   { .call = pmPowerSystem },
  [pmSysBootSetup] = { .call = pmSysBoot },
  [pmSysRunning] =   { .call = pmRunSystem },
};

void pmProcess() {
  static int lastTick = 0;
  static int lastAdcTick = 0;

  if (systickGetTick() - lastTick > TICK_BETWEEN_STATE) {
    lastTick = systickGetTick();

    if (targetState > state) {
      state++;
      if (statesFunctions[state].call) {
        statesFunctions[state].call(true);
      }
    } else if (targetState < state) {
      if (statesFunctions[state].call) {
        statesFunctions[state].call(false);
      }
      state--;
    }
  }

  // VBAT sampling can't be done to often or it will affect the reading, ~100Hz is OK.
  if (systickGetTick() - lastAdcTick > TICK_BETWEEN_ADC_MEAS && !NRF_ADC->BUSY)
  {
    uint16_t rawValue = NRF_ADC->RESULT;
    lastAdcTick = systickGetTick();

      if (adcState == adcVBAT) {
          vBat = (float) (rawValue / 1023.0) * 1.2 * ADC_SCALER * ADC_DIVIDER;
          pmStartAdc(adcISET);
      } else if (adcState == adcISET) {
          // V_ISET = I_CHARGE / 400 × R_ISET
          float v = (float) (rawValue / 1023.0) * 1.2 * 3;
          iSet = (v * 400.0) / 1000.0;
          pmStartAdc(adcVBAT);
      }
      //TODO: Handle the battery charging...
  }

}

