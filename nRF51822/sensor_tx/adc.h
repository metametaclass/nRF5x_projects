#pragma once

void adc_initialization();

int adc_start();

int adc_get_result(uint32_t *result);

void adc_reset_wakeup_marker();

int adc_is_adc_wakeup();