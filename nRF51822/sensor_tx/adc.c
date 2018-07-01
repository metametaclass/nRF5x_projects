#include "nrf.h"

#include "nrf_drv_common.h"

#include "nrf_adc.h"

#include "adc.h"

//error codes
#include "nrfs_errors.h"

#include "board_config.h"

#ifdef BOARD_CONFIG_ONE_WIRE
//start onewire
#include "ow.h"
#endif

#define ADC_IRQ_PRIORITY 2

void adc_initialization(){

  nrf_adc_config_t cfg = NRF_ADC_CONFIG_DEFAULT;
  cfg.resolution = NRF_ADC_CONFIG_RES_10BIT;
  //measure VBat
  cfg.scaling = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
  cfg.reference = NRF_ADC_CONFIG_REF_VBG;
  nrf_adc_configure(&cfg);

  nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_DISABLED);

  nrf_adc_enable();

  nrf_drv_common_irq_enable(ADC_IRQn, ADC_IRQ_PRIORITY);

  nrf_adc_int_enable(NRF_ADC_INT_END_MASK);
}

static volatile uint32_t g_adc_result = 0x80000000;

static volatile uint32_t g_adc_finished = 0;

static volatile uint32_t g_adc_wakeup = 0;


void ADC_IRQHandler(void) {
  nrf_adc_event_clear(NRF_ADC_EVENT_END);
  g_adc_result = nrf_adc_result_get();
  g_adc_finished = 2;
  g_adc_wakeup = 1;
#ifdef BOARD_CONFIG_ONE_WIRE
  onewire_start();
#endif
}

/*
int adc_start() {
  //if(g_adc_finished == 1){
    //return NRFSE_BUSY;
  //}
  //g_adc_finished = 1;
  //nrf_adc_event_clear(NRF_ADC_EVENT_END);
  //nrf_adc_start();
  return NRFSE_OK;
}
*/

int adc_get_result(uint32_t *result){
  uint32_t finished = g_adc_finished;

  if(finished == 2){
    *result = g_adc_result;
    g_adc_finished = 0;
    return NRFSE_OK;
  }
  if(finished == 1) {
    return NRFSE_BUSY;
  }
  if(finished == 0) {
    return NRFSE_NODATA;
  }
  return NRFSE_GENERIC;
}

void adc_reset_wakeup_marker(){
  g_adc_wakeup = 0;
}

int adc_is_adc_wakeup(){
  return g_adc_wakeup!=0;
}