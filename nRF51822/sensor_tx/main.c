//main device defines
#include "nrf.h"

#include <stdbool.h>

//gpio pins
#include "nrf_gpio.h"

//clock
#include "nrf_clock.h"

//irq enable
#include "nrf_drv_common.h"

//delay
#include "nrf_delay.h"

//ppi
#include "nrf_ppi.h"
//events addresses
#include "nrf_adc.h"
#include "nrf_rtc.h"


//hardware config
#include "board_config.h"

//rtc timer
#include "leds.h"


//rtc timer
#include "rtc.h"

//battery measurement
#include "adc.h"


//error codes
#include "nrfs_errors.h"

//radio transmitter
#include "radio_tx.h"


#define CLOCK_CONFIG_IRQ_PRIORITY 2


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


//nRF5_SDK_12.3.0\examples\peripheral\rtc\main.c

static void hf_clock_initialization() {
  //HFCLK consumes 1 mA and started only for read/send cycle

  /*
  //enable HF and LF start interrupts
  NRF_CLOCK->INTENSET = (CLOCK_INTENSET_LFCLKSTARTED_Enabled << CLOCK_INTENSET_LFCLKSTARTED_Pos);
  nrf_clock_int_enable(NRF_CLOCK_INT_HF_STARTED_MASK | NRF_CLOCK_INT_LF_STARTED_MASK);

  // Start 16 MHz crystal oscillator 
  //NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);

  //NRF_CLOCK->TASKS_HFCLKSTART    = 1;
  nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);

  // Wait for the external oscillator to start up 
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
    
  }*/
}


void lf_clock_initialization()
{
  //nrf_drv_common_irq_enable(POWER_CLOCK_IRQn, CLOCK_CONFIG_IRQ_PRIORITY);

  // Start low frequency crystal oscillator for RTC
  //NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;  
  nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);

  //NRF_CLOCK->TASKS_LFCLKSTART    = 1;
  nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);


  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {  
    
  }
}



void check_error(int rc){
  if(rc!=NRFSE_OK){
    led_on(BOARD_CONFIG_LED_PIN_2);
    nrf_delay_ms(250);
    led_off(BOARD_CONFIG_LED_PIN_2);
  }
}

#define RTC0_TO_HFCLK_CHANNEL 0
#define HFCLK_TO_ADC_CHANNEL 1

void ppi_init(){

  //NRF_PPI->CH[RTC0_TO_ADC_CHANNEL].EEP = (uint32_t)&NRF_RTC0->EVENTS_COMPARE[0];
  //NRF_PPI->CH[RTC0_TO_ADC_CHANNEL].TEP = (uint32_t)&NRF_ADC->TASKS_START;
  //NRF_PPI->CHENSET = (1 << RTC0_TO_ADC_CHANNEL);

  //start HFCLK on RTC0 COMPARE0
  nrf_ppi_channel_endpoint_setup(
    RTC0_TO_HFCLK_CHANNEL,
    nrf_rtc_event_address_get(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0),
    nrf_clock_task_address_get(NRF_CLOCK_TASK_HFCLKSTART)
  );
  nrf_ppi_channel_enable(RTC0_TO_HFCLK_CHANNEL);

  //start ADC on HFCLK start event
  nrf_ppi_channel_endpoint_setup(
    HFCLK_TO_ADC_CHANNEL,
    nrf_clock_event_address_get(NRF_CLOCK_EVENT_HFCLKSTARTED),
    nrf_adc_task_address_get(NRF_ADC_TASK_START)
  );  
  nrf_ppi_channel_enable(HFCLK_TO_ADC_CHANNEL);
  
}

typedef struct payload_struct_s {
  uint32_t adc;
  uint32_t wake_up_counter;  
  uint8_t error_code;
  uint8_t error_count;
} payload_struct_t;


int main(void)
{
  hf_clock_initialization();

  lf_clock_initialization();

  radio_initialization();

  adc_initialization();

  ppi_init();

  led_pin_init();

  //use LOWPWR (default value)
  //NRF_POWER->TASKS_CONSTLAT = 1;

  rtc_init(); 
    
  led_on(BOARD_CONFIG_LED_PIN_0);
  nrf_delay_ms(250);
  led_off(BOARD_CONFIG_LED_PIN_0);


  payload_struct_t payload = {0};    
  payload.error_code = 0xFF;
  int rc;

  //rc = send_packet((uint8_t*)&payload);
  //check_error(rc); 
    
  while (true)
  {
    //reset marker
    //g_rtc_wakeup = 0;
    //adc_reset_wakeup_marker();    
    led_all_off();
    __SEV();
    __WFE();
    __WFE();
#ifdef BOARD_CONFIG_LED_SIGNAL
    led_on(BOARD_CONFIG_LED_PIN_1);
#endif    
    payload.wake_up_counter++;

    //do some work
    //nrf_delay_ms(10);
    /*(g_rtc_wakeup) {
      rc = adc_start();
      if(rc!=NRFSE_OK) {
        payload.error_code = rc;
        payload.error_count++;        
      }
      rc = send_packet((uint8_t*)&payload);
      check_error(rc);
    }*/

    //check wake-up reason
    if(adc_is_adc_wakeup()){
      adc_reset_wakeup_marker();
      //rc = send_packet((uint8_t*)&payload);
      //check_error(rc); 
      rc = adc_get_result(&payload.adc);    
      payload.error_code = rc;
      rc = send_packet((uint8_t*)&payload);
      check_error(rc);       
    }    
  }
}
