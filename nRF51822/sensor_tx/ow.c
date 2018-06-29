#include "nrf.h"

#include "nrf_gpio.h"

//#include "stdbool.h"

#include "ow.h"

#include "board_config.h"

//debug output pin
#include "debug_pin.h"

// memcpy
#include <string.h> 

#include "crc8.h"

//The timer is running at 16MHz
#define TICK_PER_US 16

typedef enum {
  OW_STATE_IDLE,  
  OW_STATE_RESETTING,
  OW_STATE_EXCH_BYTE,
  OW_STATE_FINISHED,
  OW_STATE_ERROR
} onewire_state_t;

typedef enum {  
  DS18B20_STATE_IDLE,
  DS18B20_STATE_RESETTING,
  DS18B20_STATE_READ_ROM_COMMAND,
  DS18B20_STATE_READING_ROM,
  DS18B20_STATE_READ_SCRATCHPAD_COMMAND,
  DS18B20_STATE_READING_SCRATCHPAD,
  DS18B20_STATE_FINISHED,
  DS18B20_STATE_ERROR
} ds18b20_state_t;


#define ONE_WIRE_COMMAND_READ_ROM 0x33
#define ONE_WIRE_COMMAND_DS18B20_READ_SCRATCHPAD 0xBE

#define OW_MAX_RESULT_SIZE 32

static volatile int g_ow_last_error = 0;
static volatile onewire_state_t g_ow_state = OW_STATE_IDLE;
static volatile ds18b20_state_t g_ds18b20_state = DS18B20_STATE_IDLE;

static volatile int g_ow_bits = 0;
static volatile uint8_t g_ow_write = 0xFF; 
static volatile uint8_t g_ow_read = 0xFF;

static volatile int g_ow_idx;
static volatile uint8_t g_ow_result[OW_MAX_RESULT_SIZE];


void onewire_init(){  

  NRF_GPIO->PIN_CNF[BOARD_CONFIG_ONE_WIRE_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                              | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
                              | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
                              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                              | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_TIMER1->PRESCALER = 0;  

  NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk | TIMER_INTENSET_COMPARE2_Msk | TIMER_INTENSET_COMPARE3_Msk;
  NVIC_SetPriority(TIMER1_IRQn, 1);
  NVIC_EnableIRQ(TIMER1_IRQn);

  //clear timer on compare3 (stop will be processed by irq handler if needed)
  //NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE3_STOP_Msk |  TIMER_SHORTS_COMPARE3_CLEAR_Msk;

  //timeout?
  //NRF_TIMER1->PRESCALER = 8;

}


//need stop?
//need shutdown?
//need new timings?

/*typedef struct ow_timer_settings_s {
  int shutdown;
  int compare0;
} ow_timer_settings_t;*/

#define OW_TIMER_CONTINUE 0
#define OW_TIMER_STOP 1 

void onewire_goto_error(int error) {
  g_ow_last_error = error;
  g_ow_state = OW_STATE_ERROR;
  g_ds18b20_state = DS18B20_STATE_ERROR;  
}

void onewire_start_send_bit(int bit) {
  NRF_TIMER1->CC[0] = 10*TICK_PER_US; //tRec
  if(bit) {
    NRF_TIMER1->CC[1] = (10+6)*TICK_PER_US; //write 1 LOW time 6 uS
    NRF_TIMER1->CC[2] = (10+6+9)*TICK_PER_US;  //master read 15 uS
    NRF_TIMER1->CC[3] = (10+6+64)*TICK_PER_US; //tSlot 70 uS
  }else{
    NRF_TIMER1->CC[1] = (10+60)*TICK_PER_US; //write 0 LOW time 60 uS
    NRF_TIMER1->CC[2] = (10+6+9)*TICK_PER_US;  //master read 15 uS
    NRF_TIMER1->CC[3] = (10+60+10)*TICK_PER_US; //tSlot 70 uS
  }
}

void onewire_goto_exch_byte(uint8_t to_write) {
  g_ow_bits = 8;
  g_ow_write = to_write; 
  g_ow_read = 0xFF;
  g_ow_state = OW_STATE_EXCH_BYTE;  
  onewire_start_send_bit(g_ow_write & 1);
}

//process 1-wire byte exchange result
int onewire_process_byte(uint8_t read_result) {

  switch(g_ds18b20_state){
    case DS18B20_STATE_READ_ROM_COMMAND:
      g_ds18b20_state = DS18B20_STATE_READING_ROM;
      g_ow_idx = 0;
      onewire_goto_exch_byte(0xFF);
      return OW_TIMER_CONTINUE;
    case DS18B20_STATE_READING_ROM:
      if(g_ow_idx>OW_MAX_RESULT_SIZE){
        onewire_goto_error(ONE_WIRE_ERROR_TOO_LARGE_DATA);  
        return OW_TIMER_STOP;
      }
      g_ow_result[g_ow_idx] = read_result;
      g_ow_idx++;
      if(g_ow_idx<8){
        onewire_goto_exch_byte(0xFF);
        return OW_TIMER_CONTINUE;
      }
        
      uint8_t crc8 = calc_crc8_1wire((void*)g_ow_result, 8);
      if(crc8==0){
        //g_ds18b20_state = DS18B20_STATE_FINISHED;
        //g_ow_state = OW_STATE_FINISHED;        
         
        g_ds18b20_state = DS18B20_STATE_READ_SCRATCHPAD_COMMAND;
        onewire_goto_exch_byte(ONE_WIRE_COMMAND_DS18B20_READ_SCRATCHPAD);
        return OW_TIMER_CONTINUE;
      }
      onewire_goto_error(ONE_WIRE_CRC_ERROR);
      return OW_TIMER_STOP;
                
    case DS18B20_STATE_READ_SCRATCHPAD_COMMAND:
      g_ds18b20_state = DS18B20_STATE_READING_SCRATCHPAD;      
      onewire_goto_exch_byte(0xFF);
      return OW_TIMER_CONTINUE;

    case DS18B20_STATE_READING_SCRATCHPAD:
      if(g_ow_idx>OW_MAX_RESULT_SIZE){
        onewire_goto_error(ONE_WIRE_ERROR_TOO_LARGE_DATA);  
        return OW_TIMER_STOP;
      }
      g_ow_result[g_ow_idx] = read_result;
      g_ow_idx++;
      if(g_ow_idx<17){
        onewire_goto_exch_byte(0xFF);
        return OW_TIMER_CONTINUE;
      }
      crc8 = calc_crc8_1wire((void*)(g_ow_result+8), 9);
      if(crc8==0){
        g_ds18b20_state = DS18B20_STATE_FINISHED;
        g_ow_state = OW_STATE_FINISHED;        

        return OW_TIMER_STOP;
      }
      onewire_goto_error(ONE_WIRE_CRC_ERROR);
      return OW_TIMER_STOP;      
    default:
      onewire_goto_error(ONE_WIRE_ERROR_INVALID_DS18B20_STATE);
      return OW_TIMER_STOP;
  }
}



int onewire_next_state_inner(int sample){
  switch(g_ow_state){
    case OW_STATE_RESETTING:
      if(sample){
        onewire_goto_error(ONE_WIRE_ERROR_NO_DEVICES);
        return OW_TIMER_STOP;
      }
      g_ds18b20_state = DS18B20_STATE_READ_ROM_COMMAND;
      onewire_goto_exch_byte(ONE_WIRE_COMMAND_READ_ROM);
      return OW_TIMER_CONTINUE;
    case OW_STATE_EXCH_BYTE:      
      g_ow_read >>= 1;
      g_ow_read |= sample?0x80:0x00;             
      g_ow_bits--;
      if(g_ow_bits==0){
        int is_stop = onewire_process_byte(g_ow_read);
        return is_stop;
      }
      g_ow_write >>=1;
      onewire_start_send_bit(g_ow_write & 1);      
      return OW_TIMER_CONTINUE;
    case OW_STATE_ERROR:      
      return OW_TIMER_STOP;//stop timer
    default:
      onewire_goto_error(ONE_WIRE_ERROR_INVALID_STATE);      
      return OW_TIMER_STOP;
  }
}

void onewire_next_state(int sample){
  int is_stop = onewire_next_state_inner(sample);
  if(is_stop){
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_SHUTDOWN = 1;    
  }else{
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->TASKS_START = 1;
  }
}


volatile static int g_ow_sample = 0;

void TIMER1_IRQHandler(void)
{  
  if (NRF_TIMER1->EVENTS_COMPARE[0]) {    
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    if(g_ow_state==OW_STATE_IDLE || g_ow_state==OW_STATE_FINISHED || g_ow_state ==OW_STATE_ERROR){
      return;
    }
    NRF_GPIO->OUTCLR = (1 << BOARD_CONFIG_ONE_WIRE_PIN);
    debug_pin_set();
  }
  
  if (NRF_TIMER1->EVENTS_COMPARE[1]) {
    NRF_GPIO->OUTSET = (1 << BOARD_CONFIG_ONE_WIRE_PIN);
    NRF_TIMER1->EVENTS_COMPARE[1] = 0;
  }
  
  if (NRF_TIMER1->EVENTS_COMPARE[2]) {    
    g_ow_sample = (NRF_GPIO->IN >> BOARD_CONFIG_ONE_WIRE_PIN)&0x01;
    NRF_TIMER1->EVENTS_COMPARE[2] = 0;
    debug_pin_clear();
  }
  
  if (NRF_TIMER1->EVENTS_COMPARE[3]) {    
    NRF_TIMER1->EVENTS_COMPARE[3] = 0;    
    onewire_next_state(g_ow_sample);
  }
}


/*
int onewire_reset() {
  sample = 1;

  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->CC[0] = 1;
  NRF_TIMER1->CC[1] = 480*TICK_PER_US;
  NRF_TIMER1->CC[2] = (480+70)*TICK_PER_US;
  NRF_TIMER1->CC[3] = (480+480)*TICK_PER_US;


  //running = true;
  NRF_TIMER1->TASKS_START = 1;

  while(running){
    __SEV();
    __WFE();
    __WFE();
  }

  if (sample == 0)
    return 1;

  return 0;
}*/

void onewire_init_timings_reset(){
  NRF_TIMER1->CC[0] = 10;
  NRF_TIMER1->CC[1] = 480*TICK_PER_US;
  NRF_TIMER1->CC[2] = (480+70)*TICK_PER_US;
  NRF_TIMER1->CC[3] = (480+480)*TICK_PER_US;
}


void onewire_start(){
  //start FSM
  if (g_ow_state != OW_STATE_IDLE) {
    //error, still running
    //TODO: reset FSM, wait for N seconds
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_SHUTDOWN = 1;
    NRF_TIMER1->TASKS_CLEAR = 1;
    g_ow_last_error = ONE_WIRE_ERROR_NOT_IDLE;
    g_ow_state = OW_STATE_ERROR;
    return;
  }

  g_ow_idx = 0;
  g_ow_last_error = 0;
  g_ow_state = OW_STATE_RESETTING;
  g_ds18b20_state = DS18B20_STATE_RESETTING;

  NRF_TIMER1->TASKS_CLEAR = 1;
  onewire_init_timings_reset();
  NRF_TIMER1->TASKS_START = 1;

}


int onewire_read_result(uint8_t *result, size_t len, int *real_len){
  if(g_ds18b20_state == DS18B20_STATE_FINISHED){    
    memcpy(result, (void*)g_ow_result, MIN(len, g_ow_idx));
    *real_len = MIN(len, g_ow_idx);

    //DEBUG values
    /**result = 0x12;
    result++;
    *result = 0x34;
    result++;*/

    g_ds18b20_state = DS18B20_STATE_IDLE;
    g_ow_state = OW_STATE_IDLE;    
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_SHUTDOWN = 1;
    NRF_TIMER1->TASKS_CLEAR = 1;
    return 0;
  }


  if(g_ow_state == OW_STATE_ERROR) {
    if(g_ow_last_error == ONE_WIRE_CRC_ERROR) {
      memcpy(result, (void*)g_ow_result, MIN(len, g_ow_idx));
      *real_len = MIN(len, g_ow_idx);
    }
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_SHUTDOWN = 1;
    NRF_TIMER1->TASKS_CLEAR = 1;
    g_ds18b20_state = DS18B20_STATE_IDLE;
    g_ow_state = OW_STATE_IDLE;    
    return g_ow_last_error;
  }

  /*if(g_ow_state == OW_STATE_IDLE) {
    return ONE_WIRE_ERROR_NOT_STARTED;
  }*/
  
  return ONE_WIRE_ERROR_BUSY;
  
}