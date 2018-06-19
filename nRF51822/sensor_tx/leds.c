#include <stdint.h>

#include "board_config.h"

#include "nrf_gpio.h"


void led_on(uint32_t gpio){
#ifdef BOARD_CONFIG_LED_ACTIVE_LOW
  nrf_gpio_pin_clear(gpio);
#else
  nrf_gpio_pin_set(gpio);
#endif
}

void led_off(uint32_t gpio){
#ifdef BOARD_CONFIG_LED_ACTIVE_LOW
  nrf_gpio_pin_set(gpio);
#else
  nrf_gpio_pin_clear(gpio);
#endif
}

void led_pin_init(){
  nrf_gpio_cfg_output(BOARD_CONFIG_LED_PIN_0);
  led_off(BOARD_CONFIG_LED_PIN_0);

  nrf_gpio_cfg_output(BOARD_CONFIG_LED_PIN_1);
  led_off(BOARD_CONFIG_LED_PIN_1);

  nrf_gpio_cfg_output(BOARD_CONFIG_LED_PIN_2);
  led_off(BOARD_CONFIG_LED_PIN_2);
}

