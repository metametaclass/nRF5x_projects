#include "nrf.h"

#include "nrf_gpio.h"

#include "board_config.h"

#ifdef BOARD_CONFIG_DEBUG_PIN


void debug_pin_init(){
  NRF_GPIO->PIN_CNF[BOARD_CONFIG_DEBUG_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                              | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                              | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                              | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                              | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}

#endif