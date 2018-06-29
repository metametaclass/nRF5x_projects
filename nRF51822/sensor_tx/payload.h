#pragma once

#include <stdint.h>

#include "radio_tx.h"

#include <string.h>

#define PROTOCOL_VERSION_1 1

#define SENSOR_TYPE_WAKEUP 0x00
#define SENSOR_TYPE_BATTERY 0x10
#define SENSOR_TYPE_DEBUG 0x20
#define SENSOR_TYPE_DS18B20 0x30


#define SENSOR_TYPE_ERRORS 0xB0
#define SENSOR_TYPE_u8u8 0x00
#define SENSOR_TYPE_u8u16 0x01

#define SENSOR_WAKEUP 0
#define SENSOR_ADC_BATTERY 1
#define SENSOR_DEBUG 2
#define SENSOR_DS18B20 3


typedef struct payload_struct_s {
  uint8_t size;  
  union {
    uint8_t s1;
    struct {
      uint8_t no_ack :1;
      uint8_t pid :2;
    };
  };
  /*uint32_t adc;
  uint32_t wake_up_counter;  
  uint8_t error_code;
  uint8_t error_count;*/
  uint8_t data[MAX_RADIO_PAYLOAD_SIZE];
} __attribute__((packed)) payload_struct_t;


void payload_clear(payload_struct_t *payload);

void put_uint8(payload_struct_t *payload, uint8_t value);

void put_uint8_array(payload_struct_t *payload, uint8_t* data, size_t len);

void put_uint16(payload_struct_t *payload, uint16_t value);

void put_uint32(payload_struct_t *payload, uint32_t value);
