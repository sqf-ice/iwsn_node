/**
\brief General IWSN 02aMAClow-layer 'ieee802154e' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __IEEE802154E_H
#define __IEEE802154E_H

//=========================== include =========================================

#include "stdint.h"
#include "platform.h"
#include "led.h"
#include "iwsn.h"
#include "tsch.h"
#include "queue.h"
#include "stdbool.h"

//=========================== define ==========================================
#define SYNCHRONIZING_CHANNEL	 RADIO_CHANNEL // channel the node listens on to synchronize

#ifdef RADIO_CHIP_AT86RF212        //rf212
#define HEAD_CHANNEL             1 // the head channel: 1 for RF212, 11 for RF231
#define TAIL_CHANNEL            10 // the tail chnnael: 10 for RF212, 26 for RF231
#endif

#ifdef RADIO_CHIP_AT86RF231
#define HEAD_CHANNEL            11 // the head channel: 1 for RF212, 11 for RF231
#define TAIL_CHANNEL            26 // the tail chnnael: 10 for RF212, 26 for RF231
#endif

#if ( __BOARD == ANTHONY )
//10ms
#if  (SET_TS_TYPE == 0)
#define TXRETRIES       RETRY_TIME // number of retries before declaring failed
#define	KA_TXRETRIES			7  // number of KA retries before declaring failed
#define TX_POWER                31 // 1=-25dBm, 31=0dBm (max value)
#define RESYNCHRONIZATIONGUARD 150 // in 1MHz ticks. min distance to the end of the slot to succesfully synchronize
#define US_PER_TICK              1 // number of us per 1MHz clock tick
#define MONPEERTIMEOUT         500 // in slots: @10ms per slot ->  ~5 seconds
//#define MONCHILDTIMEOUT       1000 // in slots: @10ms per slot ->  ~10 seconds
//#define MONCHILDTIMEOUT       400 // in slots: @10ms per slot ->  ~10 seconds
#define MONCHILDTIMEOUT       250 // in slots: @10ms per slot ->  ~10 seconds
#define MONPARENTTIMEOUT       500 // in slots: @10ms per slot ->  ~5 seconds
//#define DESYNCTIMEOUT         1500 // in slots: @10ms per slot ->  ~15 seconds
#define DESYNCTIMEOUT         1500 // in slots: @10ms per slot ->  ~15 seconds
#define DESYNCLIMIT              3 // maximum value of looses sync before board reset
#define NUMBYTESOFSHAREDBITS    16 // number of bytes shared bits used in the adv
#define NUMOFDEDICATEDSLOTBITS   5 // number of bytes shared bits used in the adv

// Atomic durations
// expressed in 1MHz ticks:
//    - ticks = duration_in_seconds * 1000000
//    - duration_in_seconds = ticks / 1000000
enum ieee802154e_atomicdurations {
  // Time-slot related
  TsTxOffset               =  2000,
  TsLongGT                 =  1300,                 //  1300us
  TsTxAckDelay             =  2000,                 //  2000us
  TsShortGT                =  500,                  //   500us
  TsSlotDuration           =  PORT_TsSlotDuration,  // 10000us
  // Execution speed related
  maxTxDataPrepare         =  PORT_maxTxDataPrepare,
  maxRxAckPrepare          =  PORT_maxRxAckPrepare,
  maxRxDataPrepare         =  PORT_maxRxDataPrepare,
  maxTxAckPrepare          =  PORT_maxTxAckPrepare,
  // Radio speed related
  delayTx                  =  PORT_delayTx,         // between GO signal and SFD
  delayRx                  =  PORT_delayRx,         // between GO signal and start listening
  // Radio watchdog related
  wdRadioTx                =  1000,                  //  1000us (needs to be >delayTx)
  wdDataDuration           =  5000,                  //  5000us (measured 4280us with max payload)
  wdAckDuration            =  3000,                  //  3000us (measured 1000us)
};

#else
//15ms
#define TXRETRIES       RETRY_TIME // number of retries before declaring failed
#define	KA_TXRETRIES			6  // number of KA retries before declaring failed
#define TX_POWER                31 // 1=-25dBm, 31=0dBm (max value)
#define RESYNCHRONIZATIONGUARD 150 // in 1MHz ticks. min distance to the end of the slot to succesfully synchronize
#define US_PER_TICK			     1 // number of us per 1MHz clock tick
#define MONPEERTIMEOUT         500 // in slots: @10ms per slot ->  ~5 seconds
#define MONCHILDTIMEOUT       1000 // in slots: @10ms per slot ->  ~10 seconds
#define MONPARENTTIMEOUT       500 // in slots: @10ms per slot ->  ~5 seconds
#define DESYNCTIMEOUT         1500 // in slots: @10ms per slot ->  ~15 seconds
#define DESYNCLIMIT              3 // maximum value of looses sync before board reset
#define NUMBYTESOFSHAREDBITS    16 // number of bytes shared bits used in the adv

// Atomic durations
// expressed in 1MHz ticks:
//    - ticks = duration_in_seconds * 1000000
//    - duration_in_seconds = ticks / 1000000
enum ieee802154e_atomicdurations {
  // Time-slot related
  TsTxOffset               =  4000,					//4000
  TsLongGT                 =  1300,                 //  1300us
  TsTxAckDelay             =  4600,                 //  4600us
  TsShortGT                =  500,                  //   500us
  TsSlotDuration           =  PORT_TsSlotDuration,  // 15000us
  // Execution speed related
  maxTxDataPrepare         =  PORT_maxTxDataPrepare,
  maxRxAckPrepare          =  PORT_maxRxAckPrepare,
  maxRxDataPrepare         =  PORT_maxRxDataPrepare,
  maxTxAckPrepare          =  PORT_maxTxAckPrepare,
  // Radio speed related
  delayTx                  =  PORT_delayTx,         // between GO signal and SFD
  delayRx                  =  PORT_delayRx,         // between GO signal and start listening
  // Radio watchdog related
  wdRadioTx                =  1000,                  //  1000us (needs to be >delayTx)
  wdDataDuration           =  5000,                 //  5000us ()
  wdAckDuration            =  3000,                  //  3000us (measured 1000us)
};

#endif

#else

#define TXRETRIES       RETRY_TIME // number of retries before declaring failed
#define	KA_TXRETRIES			6  // number of KA retries before declaring failed
#define TX_POWER                31 // 1=-25dBm, 31=0dBm (max value)
#define RESYNCHRONIZATIONGUARD   5 // in 32kHz ticks. min distance to the end of the slot to succesfully synchronize
#define US_PER_TICK             30 // number of us per 32kHz clock tick
#define MONPEERTIMEOUT         500 // in slots: @10ms per slot ->  ~5 seconds
#define MONCHILDTIMEOUT       1000 // in slots: @10ms per slot ->  ~10 seconds
#define MONPARENTTIMEOUT       500 // in slots: @10ms per slot ->  ~5 seconds
#define DESYNCTIMEOUT         1500 // in slots: @10ms per slot ->  ~5 seconds
#define DESYNCLIMIT              3 // maximum value of looses sync before board reset
#define NUMBYTESOFSHAREDBITS    16 // number of bytes shared bits used in the adv

// Atomic durations
// expressed in 32kHz ticks:
//    - ticks = duration_in_seconds * 32768
//    - duration_in_seconds = ticks / 32768
enum ieee802154e_atomicdurations {
  // Time-slot related
  TsTxOffset               =  66,
  TsLongGT                 =  43,                   //  1300us
  TsTxAckDelay             =  66,                   //  2000us
  TsShortGT                =  16,                   //   500us
  TsSlotDuration           =  PORT_TsSlotDuration,  // 10000us
  // Execution speed related
  maxTxDataPrepare         =  PORT_maxTxDataPrepare,
  maxRxAckPrepare          =  PORT_maxRxAckPrepare,
  maxRxDataPrepare         =  PORT_maxRxDataPrepare,
  maxTxAckPrepare          =  PORT_maxTxAckPrepare,
  // Radio speed related
  delayTx                  =  PORT_delayTx,         // between GO signal and SFD
  delayRx                  =  PORT_delayRx,         // between GO signal and start listening
  // Radio watchdog related
  wdRadioTx                =   33,                  //  1000us (needs to be >delayTx)
  wdDataDuration           =  164,                  //  5000us (measured 4280us with max payload)
  wdAckDuration            =   98,                  //  3000us (measured 1000us)
};

#endif

// FSM timer durations (combinations of atomic durations)
// TX
#define DURATION_tt1 ieee802154e_vars.lastCapturedTime+TsTxOffset-delayTx-maxTxDataPrepare
#define DURATION_tt2 ieee802154e_vars.lastCapturedTime+TsTxOffset-delayTx
#define DURATION_tt3 ieee802154e_vars.lastCapturedTime+TsTxOffset-delayTx+wdRadioTx
#define DURATION_tt4 ieee802154e_vars.lastCapturedTime+wdDataDuration
#define DURATION_tt5 ieee802154e_vars.lastCapturedTime+TsTxAckDelay-TsShortGT-delayRx-maxRxAckPrepare
#define DURATION_tt6 ieee802154e_vars.lastCapturedTime+TsTxAckDelay-TsShortGT-delayRx
#define DURATION_tt7 ieee802154e_vars.lastCapturedTime+TsTxAckDelay+TsShortGT
#define DURATION_tt8 ieee802154e_vars.lastCapturedTime+wdAckDuration
// RX
#define DURATION_rt1 ieee802154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx-maxRxDataPrepare
#define DURATION_rt2 ieee802154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx
#define DURATION_rt3 ieee802154e_vars.lastCapturedTime+TsTxOffset+TsLongGT
#define DURATION_rt4 ieee802154e_vars.lastCapturedTime+wdDataDuration
#define DURATION_rt5 ieee802154e_vars.lastCapturedTime+TsTxAckDelay-delayTx-maxTxAckPrepare
#define DURATION_rt6 ieee802154e_vars.lastCapturedTime+TsTxAckDelay-delayTx
#define DURATION_rt7 ieee802154e_vars.lastCapturedTime+TsTxAckDelay-delayTx+wdRadioTx
#define DURATION_rt8 ieee802154e_vars.lastCapturedTime+wdAckDuration

#define LIMIT_PRIORITY        255
#define NUMADVSBEFORSCHEDULE  2
#define NUMADVSBEFORSYNC      3

#define NUMFRAMESFORJOINWIAT  10	//rejoin

#define ADV_PAYLOAD_LENGTH    sizeof(ieee802154e_advs_t)
#define SYNC_PAYLOAD_LENGTH   sizeof(asn_t)

#define SLOT_TABLE_LENGTH     5
#define SLOT_SUM_NUM          10

enum {
  SLOT_ADD      = 1,
  SLOT_DEL      = 2,
};

enum {
	PARENT_ADDR      = 0,
	CHILD_ADDR       = 1,
};

//=========================== typedef =========================================

typedef enum {
  S_SLEEP                   = 0x00,   // ready for next slot
  // synchronizing
  S_SYNCLISTEN              = 0x01,   // listened for packet to synchronize to network
  S_SYNCRX                  = 0x02,   // receiving packet to synchronize to network
  S_SYNCPROC                = 0x03,   // processing packet just received
  // TX
  S_TXDATAOFFSET            = 0x04,   // waiting to prepare for Tx data
  S_TXDATAPREPARE           = 0x05,   // preparing for Tx data
  S_TXDATAREADY             = 0x06,   // ready to Tx data, waiting for 'go'
  S_TXDATADELAY             = 0x07,   // 'go' signal given, waiting for SFD Tx data
  S_TXDATA                  = 0x08,   // Tx data SFD received, sending bytes
  S_RXACKOFFSET             = 0x09,   // Tx data done, waiting to prepare for Rx ACK
  S_RXACKPREPARE            = 0x0a,   // preparing for Rx ACK
  S_RXACKREADY              = 0x0b,   // ready to Rx ACK, waiting for 'go'
  S_RXACKLISTEN             = 0x0c,   // idle listening for ACK
  S_RXACK                   = 0x0d,   // Rx ACK SFD received, receiving bytes
  S_TXPROC                  = 0x0e,   // processing sent data
  // RX
  S_RXDATAOFFSET            = 0x0f,   // waiting to prepare for Rx data
  S_RXDATAPREPARE           = 0x10,   // preparing for Rx data
  S_RXDATAREADY             = 0x11,   // ready to Rx data, waiting for 'go'
  S_RXDATALISTEN            = 0x12,   // idle listening for data
  S_RXDATA                  = 0x13,   // data SFD received, receiving more bytes
  S_TXACKOFFSET             = 0x14,   // waiting to prepare for Tx ACK
  S_TXACKPREPARE            = 0x15,   // preparing for Tx ACK
  S_TXACKREADY              = 0x16,   // Tx ACK ready, waiting for 'go'
  S_TXACKDELAY              = 0x17,   // 'go' signal given, waiting for SFD Tx ACK
  S_TXACK                   = 0x18,   // Tx ACK SFD received, sending bytes
  S_RXPROC                  = 0x19,   // processing received data
  S_CANTX_READY             = 0x1a,   // Ready to TX Can message
  S_CANTX                   = 0x1b,   // RX Can message
} ieee802154e_state_t;

typedef struct {
  asn_t                 asn;                  // current absolute slot number
  
  PORT_TIMER_WIDTH      deSyncTimeout;        // how many slots left before looses sync
  uint8_t               numDeSync;            // number of looses sync
  
  bool                  isSync;               // true iff node is synchronized to network
  bool                  isWork;             // true iff node is joined to network
  
  ieee802154e_state_t   state;                // state of the FSM
  
  QueueEntry_t*         dataToSend;           // pointer to the data to send
  QueueEntry_t*         dataReceived;         // pointer to the data received
  QueueEntry_t*         ackToSend;            // pointer to the ack to send
  QueueEntry_t*         ackReceived;          // pointer to the ack received
  PORT_TIMER_WIDTH      lastCapturedTime;     // last captured time
  PORT_TIMER_WIDTH      syncCapturedTime;     // captured time used to sync
  
  uint8_t               channel;              // channel of the current slot
  uint8_t               asnOffset;
  slotOffset_t          slotOffset;           // current slot offset
  slotOffset_t          nextActiveSlotOffset; // next active slot offset
  
  uint8_t               ChannelOffset;

  uint8_t               numValidAdvs;
  uint8_t               framecount;
  uint8_t               listenForAdvs;
  uint8_t               scanSuperframe;
} ieee802154e_vars_t;

#pragma pack(1)
typedef struct {
  uint8_t               asn[5];
  uint8_t               role;
  uint8_t               channel;
  iwsn_addr_t           sinkAddr;
  iwsn_addr_t           parentAddr;
  iwsn_addr_t           childAddr;
  slotOffset_t          mySlotoffset;
  slotOffset_t          childSlotoffset;
  uint8_t               dSlotTable[SLOT_TABLE_LENGTH];
}__attribute__ ((packed)) ieee802154e_advs_t;
#pragma pack()


// IEEE802.15.4E acknowledgement (ACK)
typedef struct {
  PORT_SIGNED_INT_WIDTH timeCorrection;
} IEEE802154E_ACK_ht;

typedef struct nodeTable
{
	uint8_t slotOffset;
	uint8_t index;
} nodeSlotTable_t;


#ifdef DEBUG_TIRI
typedef struct{
	uint16_t ti2A;
	uint16_t ti2B;
	uint16_t ti3A;
	uint16_t ti3B;
	uint16_t ti4A;
	uint16_t ti4B;
	uint16_t ti5A;
	uint16_t ti5B;
	uint16_t ti6A;
	uint16_t ti6B;
	uint16_t ti7A;
	uint16_t ti7B;
	uint16_t ti8A;
	uint16_t ti8B;
	uint16_t ti9A;
	uint16_t ti9B;
	uint16_t ti5e;
}ti_t;

typedef struct{
	uint16_t ri2A;
	uint16_t ri2B;
	uint16_t ri3A;
	uint16_t ri3B;
	uint16_t ri4A;
	uint16_t ri4B;
	uint16_t ri5A;
	uint16_t ri5B;
	uint16_t ri6A;
	uint16_t ri6B;
	uint16_t ri7A;
	uint16_t ri7B;
	uint16_t ri8A;
	uint16_t ri8B;
	uint16_t ri9A;
	uint16_t ri9B;

}ri_t;

#endif








//=========================== variables =======================================

//=========================== prototypes ======================================

void      ieee802154e_init();
bool      ieee802154e_isSynch();
bool      ieee802154e_isWork();
bool 	  ieee802154e_isNeighborScand();
void      ieee802154e_changeIsWork(bool isWork);
void 	  ieee802154e_changeIsSync(bool newIsSync);
asn_t*    ieee802154e_getCurASN();

void      isr_ieee802154e_newSlot();
void      isr_ieee802154e_timer();
void      isr_ieee802154e_startOfFrame(PORT_TIMER_WIDTH capturedTime);
void      isr_ieee802154e_endOfFrame(PORT_TIMER_WIDTH capturedTime);

void ieee802154e_storeAssignedTsch(slotOffset_t MonSlotOffset,
                                   slotOffset_t ADVSlotOffset,
                                   slotOffset_t DedicatedSlotOffset,
                                   uint8_t ChannelOffset);   //rev.2

void      ieee802154e_updateJoinPriorityForAdv(uint8_t action);
void      ieee802154e_updateDedicatedSlotForAdv(uint8_t action, slotOffset_t slotOffset);  //rev.2
void 	  ieee802154e_updateNodeSlotInTable(uint8_t type, iwsn_addr_t* address, slotOffset_t slot);  //v3
bool      ieee802154e_isDedicatedSlot(uint16_t index);	//rev.2
void      ieee802154e_setUnDedicatedSlotForAdv(uint16_t index);	//rev.2
void      ieee802154e_setDedicatedSlotForAdv(uint16_t index);	//rev.2
bool 	  ieee802154e_isMyDedicatedSlotCanceled(uint16_t index);	//rev.2
void      ieee802154e_prepareAdvForOthers();
uint8_t   ieee802154e_getJoinChannel();
uint8_t   ieee802154e_getCurLevel();
uint8_t   ieee802154e_getCurPriority();
void      ieee802154e_notifyChangeIsJoined(bool newIsJoined);
void      ieee802154e_listenForAdvs();
uint8_t*  ieee802154e_getSlotTable();
void      ieee802154e_ledsCanProcess();  //leds_can
uint8_t   ieee802154e_getMyLevel();		//rev.2
uint8_t   ieee802154e_getMyPriority();	//rev.2
slotOffset_t ieee802154e_getMyDedicatedSlot();	//rev.2
void 	  ieee802154e_updateAdvAddrInfo(uint8_t addrType, iwsn_addr_t* addr);  //v3
void      ieee802154e_updateAdvInfo();  //v3
#endif
