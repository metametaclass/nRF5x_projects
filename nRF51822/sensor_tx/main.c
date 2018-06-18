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


#define RTC_PRESCALER 4095 
#define RTC_TICK_PER_SECOND 8
#define RTC_COMPARE_TICKS (RTC_TICK_PER_SECOND*5)

void clock_initialization()
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART    = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
      
  }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
  NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  //NRF_CLOCK->INTENSET = (CLOCK_INTENSET_LFCLKSTARTED_Enabled << CLOCK_INTENSET_LFCLKSTARTED_Pos);
  NRF_CLOCK->TASKS_LFCLKSTART    = 1;

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
}


void RTC0_IRQHandler(void)
{
  //from nrf_drv_rtc handler, why so complex?
  if( nrf_rtc_int_is_enabled(NRF_RTC0, NRF_RTC_INT_TICK_MASK) &&
      nrf_rtc_event_pending (NRF_RTC0, NRF_RTC_EVENT_TICK)) {
    nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_TICK);
  
    //nrf_gpio_pin_clear(BOARD_CONFIG_LED_PIN_0);
  }
  //NRF_RTC0 -> EVENTS_TICK = 0;


  if( nrf_rtc_int_is_enabled(NRF_RTC0, NRF_RTC_INT_COMPARE0_MASK) &&
      nrf_rtc_event_pending (NRF_RTC0, NRF_RTC_EVENT_COMPARE_0)) {
    //reset event
    nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0);    

    //set new value
    uint32_t val = nrf_rtc_counter_get(NRF_RTC0);
    val = RTC_WRAP((val + RTC_COMPARE_TICKS));
    nrf_rtc_cc_set(NRF_RTC0, 0, val);

    led_on(BOARD_CONFIG_LED_PIN_0);
  }

  
  //nrf_gpio_pin_toggle(BOARD_CONFIG_LED_PIN_1);
  //nrf_drv_rtc_int_handler(NRF_RTC0,RTC0_INSTANCE_INDEX, NRF_RTC_CC_CHANNEL_COUNT(0));
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
  nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);
 
  NRF_RTC0 -> POWER = 1;
  nrf_rtc_prescaler_set(NRF_RTC0, RTC_PRESCALER);  

  nrf_rtc_cc_set(NRF_RTC0, 0, RTC_COMPARE_TICKS);

  nrf_drv_common_irq_enable(RTC0_IRQn, 7);
  
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

  led_pin_init();

  //NRF_POWER->TASKS_CONSTLAT = 1;//TODO: replace with LOWPWR (default value)

  rtc_init(); 
    
  led_on(BOARD_CONFIG_LED_PIN_0);
  nrf_delay_ms(250);
  led_off(BOARD_CONFIG_LED_PIN_0);
    
  while (true)
  {
    led_off(BOARD_CONFIG_LED_PIN_0);
    led_off(BOARD_CONFIG_LED_PIN_1);
    __SEV();
    __WFE();
    __WFE();
    led_on(BOARD_CONFIG_LED_PIN_1);


    //do some work
    nrf_delay_ms(10);
  }
}
