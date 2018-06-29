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

//leds
#include "leds.h"

//debug output pin
#include "debug_pin.h"

//rtc timer
#include "rtc.h"

//battery measurement
#include "adc.h"

//1-wire 
#include "ow.h"

//error codes
#include "nrfs_errors.h"

//radio transmitter
#include "radio_tx.h"

//payload packet structure
#include "payload.h"

//main working loop context structure type
#include "main_context.h"

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
#ifdef BOARD_CONFIG_LED_SIGNAL    
    led_on(BOARD_CONFIG_LED_PIN_2);
    nrf_delay_ms(250);
    led_off(BOARD_CONFIG_LED_PIN_2);
#endif    
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


int fill_payload(payload_struct_t *payload, main_context_t *ctx){
  int rc;

  //build payload packet

  //Packet IDentifier field
  //ctx->packet_counter++;
  //ctx->packet_counter &= 0x03;
  //payload->pid = ctx->packet_counter;

  payload->pid += 1;

  payload->no_ack = 1;
  payload->size = 0;
  uint8_t errors[32];
  uint8_t error_count = 0;
  

  put_uint8(payload, PROTOCOL_VERSION_1);
  put_uint16(payload, (uint16_t) (NRF_FICR->DEVICEID[0]));

  uint32_t adc;
  rc = adc_get_result(&adc);
  
  if(rc){
    errors[2*error_count] = SENSOR_ID_ADC_BATTERY;
    errors[2*error_count+1] = rc;
    error_count++;
    //put_uint8(payload, SENSOR_TYPE_ERRORS | 1); //type:B, count:1  
    //put_uint8(payload, SENSOR_ID_ADC_BATTERY);
    //put_uint8(payload, (uint8_t)rc);
  }else{
    //put_uint8(payload, SENSOR_TYPE_u8u16 | 1); //type:1, count:1
    //put_uint8(payload, SENSOR_ADC_BATTERY);
    put_uint8(payload, SENSOR_TYPE_BATTERY);
    put_uint16(payload, (uint16_t)adc);
  }

  /*put_uint8(payload, SENSOR_TYPE_u8u8 | 1) //type:0, count:1
  put_uint8(payload, SENSOR_WAKEUP);
  put_uint8(payload, wake_up_counter);      */
  put_uint8(payload, SENSOR_TYPE_WAKEUP);//well-known sensor type
  put_uint8(payload, ctx->wake_up_counter & 0xFF);

  //put_uint8(payload, SENSOR_TYPE_DEBUG);//well-known sensor type
  //put_uint8(payload, ctx->one_wire_error);  

  if(ctx->one_wire_error!=0){
    errors[2*error_count] = SENSOR_ID_DS18B20;
    errors[2*error_count+1] = ctx->one_wire_error;
    error_count++;    
  } else {
    put_uint8(payload, SENSOR_TYPE_DS18B20);
    put_uint8_array(payload, ctx->onewire_rom, 8);
    put_uint16(payload, ctx->onewire_rom[8]+(ctx->onewire_rom[9]<<8));

    //scratchpad
    //ctx->onewire_rom[13] = ctx->real_len;
    //put_uint8(payload, SENSOR_TYPE_BYTE_ARRAY | 6);
    //put_uint8_array(payload, ctx->onewire_rom+8, 6);
  }
  if (error_count>0) {
    //type:B, count:error_count 
    put_uint8(payload, SENSOR_TYPE_ERRORS | error_count); 
    for(int i=0;i<error_count*2;i++){
      //put id, error
      put_uint8(payload, errors[i]);      
    }
  }
  return 0;
}



int main(void)
{
  hf_clock_initialization();

  lf_clock_initialization();

  radio_initialization();

  adc_initialization();

  ppi_init();

  led_pin_init();

  debug_pin_init();

  onewire_init();

  //use LOWPWR (default value)
  //NRF_POWER->TASKS_CONSTLAT = 1;

  rtc_init(); 

  payload_struct_t payload = {0};      
  main_context_t ctx = {0};
    
  led_on(BOARD_CONFIG_LED_PIN_0);
  nrf_delay_ms(250);  
  led_off(BOARD_CONFIG_LED_PIN_0);


  
  int rc;
    
  while (true)
  {
    //reset marker
    //g_rtc_wakeup = 0;
    //adc_reset_wakeup_marker();    
#ifdef BOARD_CONFIG_LED_SIGNAL    
    led_all_off();
    debug_pin_clear();
#endif    
    __SEV();
    __WFE();
    __WFE();
#ifdef BOARD_CONFIG_LED_SIGNAL
    led_on(BOARD_CONFIG_LED_PIN_1);
    debug_pin_set();
#endif    
    ctx.wake_up_counter++;

    //check wake-up reason
    if(!adc_is_adc_wakeup()){
      continue;//
    }
    int real_len = 0;
    rc = onewire_read_result(ctx.onewire_rom, sizeof(ctx.onewire_rom), &real_len);
    if(rc==ONE_WIRE_ERROR_BUSY){
      continue;
    }
    ctx.one_wire_error = rc;
    ctx.real_len = (uint8_t)real_len;

    adc_reset_wakeup_marker();
    rc = fill_payload(&payload, &ctx);
    check_error(rc);

    rc = send_packet((uint8_t*)&payload);
    check_error(rc);  
    //nrf_delay_ms(1000);      

  }
}


