//main device defines
#include "nrf.h"

#include <stdbool.h>

//gpio pins
#include "nrf_gpio.h"

//clock
#include "nrf_clock.h"

//irq enable
#include "nrf_drv_common.h"

//rtc
#include "nrf_rtc.h"

//gpio pins
#include "nrf_delay.h"

//hardware config
#include "board_config.h"

#include "radio_tx.h"

//32768/4096
#define RTC_PRESCALER 4095 

//8 ticks per second 
#define RTC_TICK_PER_SECOND 8

//main wake-up event interval
#define RTC_COMPARE_TICKS (RTC_TICK_PER_SECOND*5)

#define CLOCK_CONFIG_IRQ_PRIORITY 2

#define RTC_DEFAULT_CONFIG_IRQ_PRIORITY 3

void POWER_CLOCK_IRQHandler(void){
  if (nrf_clock_event_check(NRF_CLOCK_EVENT_HFCLKSTARTED)) {
    nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_int_disable(NRF_CLOCK_INT_HF_STARTED_MASK);
  }

  if (nrf_clock_event_check(NRF_CLOCK_EVENT_LFCLKSTARTED)) {
    nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);        
    nrf_clock_int_disable(NRF_CLOCK_INT_LF_STARTED_MASK);
  }

}


void clock_initialization()
{
  //nrf_drv_common_irq_enable(POWER_CLOCK_IRQn, CLOCK_CONFIG_IRQ_PRIORITY);

  //enable HF and LF start interrupts
  //NRF_CLOCK->INTENSET = (CLOCK_INTENSET_LFCLKSTARTED_Enabled << CLOCK_INTENSET_LFCLKSTARTED_Pos);
  //nrf_clock_int_enable(NRF_CLOCK_INT_HF_STARTED_MASK | NRF_CLOCK_INT_LF_STARTED_MASK);

  /* Start 16 MHz crystal oscillator */

  //NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);

  //NRF_CLOCK->TASKS_HFCLKSTART    = 1;
  nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
    
  }

    /* Start low frequency crystal oscillator for RTC*/
  

  //NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;  
  nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);

  //NRF_CLOCK->TASKS_LFCLKSTART    = 1;
  nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);


  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
  {  
    
  }
}

//nRF5_SDK_12.3.0\examples\peripheral\rtc\main.c

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

volatile uint32_t g_counter;
volatile uint32_t g_rtc_wakeup;

void RTC0_IRQHandler(void)
{
  //from nrf_drv_rtc handler, why so complex?
  if( nrf_rtc_int_is_enabled(NRF_RTC0, NRF_RTC_INT_TICK_MASK) &&
      nrf_rtc_event_pending (NRF_RTC0, NRF_RTC_EVENT_TICK)) {
    nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_TICK);
      
    led_on(BOARD_CONFIG_LED_PIN_1);
  }

  if( nrf_rtc_int_is_enabled(NRF_RTC0, NRF_RTC_INT_COMPARE0_MASK) &&
      nrf_rtc_event_pending (NRF_RTC0, NRF_RTC_EVENT_COMPARE_0)) {
    //reset event
    nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0);    

    //set new value
    uint32_t val = nrf_rtc_counter_get(NRF_RTC0);
    g_counter = val;//copy counter
    g_rtc_wakeup = 1;//wake up marker
    val = RTC_WRAP((val + RTC_COMPARE_TICKS));
    nrf_rtc_cc_set(NRF_RTC0, 0, val);

    led_on(BOARD_CONFIG_LED_PIN_0);
  }

}


/*

    event = NRF_RTC_EVENT_TICK;
    if (nrf_rtc_int_is_enabled(p_reg,NRF_RTC_INT_TICK_MASK) &&
        nrf_rtc_event_pending(p_reg, event))
    {
        nrf_rtc_event_clear(p_reg, event);
        NRF_LOG_DEBUG("Event: %s, instance id: %d.\r\n", (uint32_t)EVT_TO_STR(event), instance_id);
        m_handlers[instance_id](NRF_DRV_RTC_INT_TICK);
    }

*/


void rtc_init(){  
  //NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);  
  nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);
 
  NRF_RTC0 -> POWER = 1;
  nrf_rtc_prescaler_set(NRF_RTC0, RTC_PRESCALER);  

  nrf_rtc_cc_set(NRF_RTC0, 0, RTC_COMPARE_TICKS);

  nrf_drv_common_irq_enable(RTC0_IRQn, RTC_DEFAULT_CONFIG_IRQ_PRIORITY);
  
  nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_TICK);
  nrf_rtc_event_disable(NRF_RTC0, NRF_RTC_EVENT_TICK);

  nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_event_disable(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0);

  nrf_rtc_int_enable(NRF_RTC0, /*NRF_RTC_INT_TICK_MASK |*/ NRF_RTC_INT_COMPARE0_MASK); 

  nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_START);
  
}


int main(void)
{
  clock_initialization();

  radio_initialization();

  led_pin_init();

  //NRF_POWER->TASKS_CONSTLAT = 1;//TODO: replace with LOWPWR (default value)

  rtc_init(); 
    
  led_on(BOARD_CONFIG_LED_PIN_0);
  nrf_delay_ms(250);
  led_off(BOARD_CONFIG_LED_PIN_0);


    
  while (true)
  {
    //reset marker
    g_rtc_wakeup = 0;

    led_off(BOARD_CONFIG_LED_PIN_0);
    led_off(BOARD_CONFIG_LED_PIN_1);
    led_off(BOARD_CONFIG_LED_PIN_2);
    __SEV();
    __WFE();
    __WFE();
    led_on(BOARD_CONFIG_LED_PIN_1);

    //do some work
    //nrf_delay_ms(10);
    if(g_rtc_wakeup) {
      int32_t rc = send_packet((uint8_t*)&g_counter);
      if(rc) {
        led_off(BOARD_CONFIG_LED_PIN_0);
        led_off(BOARD_CONFIG_LED_PIN_1);
        led_on(BOARD_CONFIG_LED_PIN_2);
        nrf_delay_ms(250);
        led_off(BOARD_CONFIG_LED_PIN_2);
      }
    }
  }
}
