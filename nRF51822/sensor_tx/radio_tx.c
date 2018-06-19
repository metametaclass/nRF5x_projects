#include <stdint.h>
#include "radio_tx.h"

//device defines
#include "nrf.h"

//irq control
#include "nrf_drv_common.h"

//error codes
#include "nrfs_errors.h"

//schedule next compare event on radio finish
#include "rtc.h"

#include "board_config.h"


#define BOARD_CONFIG_RADIO_CHANNEL 13

#define RADIO_IRQ_PRIORITY 2


/* These are set to zero as ShockBurst packets don't have corresponding fields. */
#define PACKET_S1_FIELD_SIZE      (0UL)  /**< Packet S1 field size in bits. */
#define PACKET_S0_FIELD_SIZE      (0UL)  /**< Packet S0 field size in bits. */
#define PACKET_LENGTH_FIELD_SIZE  (0UL)  /**< Packet length field size in bits. */

#define PACKET_BASE_ADDRESS_LENGTH  (4UL)                   //!< Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH        (10UL)                   //!< Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE      (PACKET_STATIC_LENGTH)  //!< Packet payload maximum size in bytes


#ifdef USE_NRF_EXAMPLE_RF_SETUP
//nordic\nRF5_SDK_12.3.0\components\drivers_nrf\radio_config\radio_config.c
static void radio_address_config_nrf_example(){
  NRF_RADIO->FREQUENCY = 7;  
  
  // Radio address config
  NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format

  NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

  NRF_RADIO->BASE0 = bytewise_bitswap(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
  NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format
}

#else 

//our address and channel
static void radio_address_config_sensor_tx(){
  NRF_RADIO->FREQUENCY = BOARD_CONFIG_RADIO_CHANNEL;  

  // Radio address config
  NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xB3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xB2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xB1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xB0) << 0); // Prefix byte of address 0 converted to nRF24L series format

  NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xB7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xB6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xB4) << 0); // Prefix byte of address 4 converted to nRF24L series format

  NRF_RADIO->BASE0 = bytewise_bitswap(0x02143657UL);  // Base address for prefix 0 converted to nRF24L series format
  NRF_RADIO->BASE1 = bytewise_bitswap(0x98BADCFEUL);  // Base address for prefix 1-7 converted to nRF24L series format
}
#endif


void radio_initialization() {

  NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);  
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

#ifdef USE_NRF_EXAMPLE_RF_SETUP
  radio_address_config_nrf_example();
#else
  radio_address_config_sensor_tx();
#endif

  //address: B0 02 14 36 57
  NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
  //NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

 // Packet configuration
  NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                     (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
                     (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

  // Packet configuration
  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                     (PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |
                     (PACKET_STATIC_LENGTH         << RADIO_PCNF1_STATLEN_Pos) |
                     (PACKET_PAYLOAD_MAXSIZE       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // CRC Config
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
  if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos)) {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
  }    
  else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos)){
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
  }


  //shortcuts ready -> start, end -> disable
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
                      RADIO_SHORTS_END_DISABLE_Msk; 

  //interrupt on finish radio cycle
  NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;

  nrf_drv_common_irq_enable(RADIO_IRQn, RADIO_IRQ_PRIORITY);
}

volatile uint32_t g_radio_active;

void RADIO_IRQHandler(){

  if (NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk)) {
    NRF_RADIO->EVENTS_DISABLED = 0;
    rtc_schedule_next_event();
    g_radio_active = 0;
  }
}


int send_packet(uint8_t *packet){ 

  // start TX sequence
  if(g_radio_active) {
    //do nothing
    return NRFSE_BUSY;
  }

  g_radio_active = 1;

  NRF_RADIO->PACKETPTR = (uint32_t)packet;  

  NRF_RADIO->EVENTS_READY = 0U;
  NRF_RADIO->EVENTS_DISABLED = 0U;
  NRF_RADIO->TASKS_TXEN   = 1;  

  return NRFSE_OK;
}
