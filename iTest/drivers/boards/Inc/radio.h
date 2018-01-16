/*
 *	File: radio.h
 *	Description:
 *		Peach-specific definition of the "radio"
 *	Created on: 2013-7-30
 *	Author: 
 *		Dong Yang 		<dyang@bjtu.edu.cn>
 *		Hongchao Wang 	<hcwang@bjtu.edu.cn>
 *		Hao Zhang 		<zh1047@gmail.com>
 */

#ifndef RADIO_H_
#define RADIO_H_

//=========================== include =========================================

#include "platform.h"

//=========================== define ==========================================
#define LENGTH_CRC 2

enum {
  RADIO_CCA_ED                    = 0,  // Use energy detection above threshold mode
  RADIO_CCA_CARRIER_SENSE         = 1,  // Use carrier sense mode
  RADIO_CCA_CARRIER_SENSE_WITH_ED = 2,  // Use a combination of both energy detection and carrier sense
  RADIO_CCA_CARRIER_SENSE_AND_ED = 3,  // Use a combination of both energy detection and carrier sense
};  //CCA

#define RADIO_MAX_NUM_CCA        PORT_RADIO_MAX_NUM_CCA     //CCA
#define RADIO_MAX_ED_THRESHOLD   PORT_RADIO_MAX_ED_THRESHOLD   //CCA
//=========================== typedef =========================================

/**
\brief Current state of the radio.

\note This radio driver is very minimal in that it does not follow a state machine.
      It is up to the MAC layer to ensure that the different radio operations
      are called in the right order. The radio keeps a state for debugging purposes only.
*/

typedef enum {
	RADIOSTATE_STOPPED             = 0x00,   ///< Completely stopped.
	RADIOSTATE_RFOFF               = 0x01,   ///< Listening for commands by RF chain is off.
	RADIOSTATE_SETTING_CHANNEL     = 0x02,   ///< Configuring the frequency.
	RADIOSTATE_CHANNEL_SET         = 0x03,   ///< Done configuring the frequency.
	RADIOSTATE_LOADING_PACKET      = 0x04,   ///< Loading packet to send over SPI.
	RADIOSTATE_PACKET_LOADED       = 0x05,   ///< Packet is loaded in the TX buffer.
	RADIOSTATE_ENABLING_TX         = 0x06,   ///< The RF Tx chaing is being enabled (includes locked the PLL).
	RADIOSTATE_TX_ENABLED          = 0x07,   ///< Radio completely ready to transmit.
	RADIOSTATE_TRANSMITTING        = 0x08,   ///< Busy transmitting bytes.
	RADIOSTATE_ENABLING_RX         = 0x09,   ///< The RF Rx chaing is being enabled (includes locked the PLL).
	RADIOSTATE_LISTENING           = 0x0a,   ///< RF chain is on, listening, but no packet received yet.
	RADIOSTATE_RECEIVING           = 0x0b,   ///< Busy receiving bytes.
	RADIOSTATE_TXRX_DONE           = 0x0c,   ///< Frame has been sent/received completely.
	RADIOSTATE_TURNING_OFF         = 0x0d,   ///< Turning the RF chain off.
} radio_state_t;

typedef uint8_t radio_trx_status_t;
typedef void (*radio_capture_cbt)(PORT_TIMER_WIDTH timestamp);

typedef struct {
  uint8_t              tx_buffer[125];
  uint8_t              rx_buffer[129];
  radio_capture_cbt    startFrame_cb;
  radio_capture_cbt    endFrame_cb;
  radio_state_t        state;
  radio_trx_status_t   trx_status;
} radio_vars_t;


//=========================== variables =======================================

//=========================== prototypes ======================================

void     radio_init();
void     radio_setStartFrameCb(radio_capture_cbt cb);
void     radio_setEndFrameCb(radio_capture_cbt cb);
void     radio_reset();
void     radio_setChannel(uint8_t channel);
void     radio_rfOn();
void     radio_rfOff();
void     radio_loadPacket(uint8_t* packet, uint8_t len);
void     radio_txEnable();
void     radio_txNow();
void     radio_rxEnable();
void     radio_rxNow();
void     radio_getReceivedFrame(uint8_t* bufRead,
                                uint8_t* lenRead,
                                uint8_t  maxBufLen,
                                 int8_t* rssi,
                                uint8_t* lqi,
                                uint8_t* crc);

uint8_t	 radio_isr();

#endif /* RADIO_H_ */
