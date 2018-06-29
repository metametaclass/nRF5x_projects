#include "nrf.h"

#include "nrf_gpio.h"

#include "stdbool.h"

#include "board_config.h"

//The timer is running at 16MHz
#define TICK_PER_US 16

void onewire_init(){
  NRF_GPIO->PIN_CNF[ONE_WIRE_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                              | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
                              | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
                              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                              | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_TIMER1->PRESCALER = 0;

  NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk | TIMER_INTENSET_COMPARE2_Msk | TIMER_INTENSET_COMPARE3_Msk;
  NVIC_SetPriority(TIMER1_IRQn, 1);
  NVIC_EnableIRQ(TIMER1_IRQn);

  //Compare 3 shorts to stop
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE3_STOP_Msk | TIMER_SHORTS_COMPARE3_CLEAR_Msk;

}

static bool running = false;
static int sample = 0;

void TIMER1_IRQHandler(void)
{
  if (NRF_TIMER1->EVENTS_COMPARE[0]) {
    NRF_GPIO->OUTCLR = (1 << ONE_WIRE_PIN);
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
  }
  
  if (NRF_TIMER1->EVENTS_COMPARE[1]) {
    NRF_GPIO->OUTSET = (1 << ONE_WIRE_PIN);
    NRF_TIMER1->EVENTS_COMPARE[1] = 0;
  }
  
  if (NRF_TIMER1->EVENTS_COMPARE[2]) {
    sample = (NRF_GPIO->IN >> ONE_WIRE_PIN)&0x01;
    NRF_TIMER1->EVENTS_COMPARE[2] = 0;
  }
  
  if (NRF_TIMER1->EVENTS_COMPARE[3]) {
    running = false;
    NRF_TIMER1->EVENTS_COMPARE[3] = 0;
  }
}


int onewire_reset() {
  sample = 1;

  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->CC[0] = 1;
  NRF_TIMER1->CC[1] = 480*TICK_PER_US;
  NRF_TIMER1->CC[2] = (480+70)*TICK_PER_US;
  NRF_TIMER1->CC[3] = (480+480)*TICK_PER_US;

  running = true;
  NRF_TIMER1->TASKS_START = 1;

  while(running){
    __SEV();
    __WFE();
    __WFE();
  }

  if (sample == 0)
    return 1;

  return 0;
}


void onewire_start(){
  //start FSM
}