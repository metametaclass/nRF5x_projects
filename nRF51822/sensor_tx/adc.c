#include "nrf.h"

#include "nrf_drv_common.h"

#include "nrf_adc.h"

//error codes
#include "nrfs_errors.h"

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

volatile uint32_t g_adc_result = 0x80000000;

volatile uint32_t g_adc_finished = 0;

void ADC_IRQHandler(void) {
  nrf_adc_event_clear(NRF_ADC_EVENT_END);
  g_adc_result = nrf_adc_result_get();
  g_adc_finished = 2;
}

int adc_start() {
  if(g_adc_finished == 1){
    return NRFSE_BUSY;
  }
  g_adc_finished = 1;
  nrf_adc_event_clear(NRF_ADC_EVENT_END);
  nrf_adc_start();
  return NRFSE_OK;
}

int adc_get_result(uint32_t *result){
  //uint32_t finished = g_adc_finished;
  *result = g_adc_result;
  g_adc_finished = 0;
  return 0;

  /*if(finished == 2){
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
  return NRFSE_GENERIC;*/
}