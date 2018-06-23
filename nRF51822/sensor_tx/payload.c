#include "payload.h"


void payload_clear(payload_struct_t *payload){
	payload->size = 0;
	payload->pid++;
}




void put_uint8(payload_struct_t *payload, uint8_t value){
  if(payload->size+sizeof(value) > MAX_RADIO_PAYLOAD_SIZE)  {
    return;
  }
  memcpy(&payload->data[payload->size], &value, sizeof(value));
  payload->size += sizeof(value);
}

void put_uint16(payload_struct_t *payload, uint16_t value){
  if(payload->size+sizeof(value) > MAX_RADIO_PAYLOAD_SIZE)  {
    return;
  }
  memcpy(&payload->data[payload->size], &value, sizeof(value));
  payload->size += sizeof(value);	
}

void put_uint32(payload_struct_t *payload, uint32_t value){
  if(payload->size+sizeof(value) > MAX_RADIO_PAYLOAD_SIZE)  {
    return;
  }
  memcpy(&payload->data[payload->size], &value, sizeof(value));
  payload->size += sizeof(value);
}
