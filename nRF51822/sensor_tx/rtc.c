#include "nrf.h"


#include "nrf_clock.h"

//rtc
#include "nrf_rtc.h"

//hardware config
#include "board_config.h"

#include "leds.h"

//32768/4096
#define RTC_PRESCALER 4095 

//8 ticks per second 
#define RTC_TICK_PER_SECOND 8

#ifndef BOARD_CONFIG_SEND_INTERVAL
#define BOARD_CONFIG_SEND_INTERVAL 5
#endif

//main wake-up event interval
#define RTC_COMPARE_TICKS (RTC_TICK_PER_SECOND*BOARD_CONFIG_SEND_INTERVAL)

#define RTC_DEFAULT_CONFIG_IRQ_PRIORITY 3


void rtc_init(){  
  //NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);  
  nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);
 
  NRF_RTC0 -> POWER = 1;
  nrf_rtc_prescaler_set(NRF_RTC0, RTC_PRESCALER);  

  nrf_rtc_cc_set(NRF_RTC0, 0, RTC_COMPARE_TICKS);  
  
  nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_TICK);
  nrf_rtc_event_disable(NRF_RTC0, RTC_EVTENSET_TICK_Msk);

  nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0);
  nrf_rtc_event_enable(NRF_RTC0, RTC_EVTENSET_COMPARE0_Msk);

  //nrf_drv_common_irq_enable(RTC0_IRQn, RTC_DEFAULT_CONFIG_IRQ_PRIORITY);
  //nrf_rtc_int_enable(NRF_RTC0, /*NRF_RTC_INT_TICK_MASK |*/ NRF_RTC_INT_COMPARE0_MASK); 

  nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_START);

  //first start in 1 second
  uint32_t val = nrf_rtc_counter_get(NRF_RTC0);
  val = RTC_WRAP((val + RTC_TICK_PER_SECOND));
  nrf_rtc_cc_set(NRF_RTC0, 0, val);
  
}

void rtc_schedule_next_event(){
  uint32_t val = nrf_rtc_counter_get(NRF_RTC0);
  val = RTC_WRAP((val + RTC_COMPARE_TICKS));
  nrf_rtc_cc_set(NRF_RTC0, 0, val);
}

//irq not used, RTC COMPARE0 event starts HFCLK through PPI

/*
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
    g_counter = nrf_rtc_counter_get(NRF_RTC0);//copy counter
    g_rtc_wakeup = 1;//wake up marker
    rtc_schedule_next_event();

    led_on(BOARD_CONFIG_LED_PIN_0);
  }

}
*/