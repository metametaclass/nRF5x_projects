/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file blinky.c (changed by pcbreflux)
 * @brief Blinky Example Application main file.
 *
 */
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"


#if SIMPLE_BLINK==0
#undef SIMPLE_BLINK
#endif


const uint32_t led_pin1 = BSP_LED_0;

/**
 * @brief Function for application main entry.
 */
int main(void) {


#ifdef SIMPLE_BLINK
    //simple blink with direct pin configuration
    // setup
    // Configure LED-pin as outputs and clear.
    nrf_gpio_cfg_output(led_pin1);
    nrf_gpio_pin_clear(led_pin1);

    // loop
    // Toggle LED.
    while (true) {
        nrf_gpio_pin_toggle(led_pin1);
        nrf_delay_ms(500);
    }
#else

    //nordic example with boards/bsp configuration

    /* Configure board. */
    bsp_board_leds_init();

    /* Toggle LEDs. */
    while (true)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            bsp_board_led_invert(i);
            nrf_delay_ms(500);
        }
    }

#endif
}



/** @} */
