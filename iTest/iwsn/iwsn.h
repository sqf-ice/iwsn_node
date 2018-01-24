/**
\brief General IWSN core "iwsn" declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, Created on: 2013-4-9.
*/

#ifndef IWSN_H_
#define IWSN_H_
//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================

enum {
  E_SUCCESS                           = 0,
  E_FAIL                              = 1,
};

// types of addresses
typedef enum {
  ADDR_NONE                           = 0,
  ADDR_16B                            = 1,
  ADDR_64B                            = 2,
  ADDR_128B                           = 3,
  ADDR_PANID                          = 4,
  ADDR_PREFIX                         = 5,
  ADDR_ANYCAST                        = 6,
} iwsn_addr_type_t;

enum {
  LITTLE_ENDIAN                       = 1,
  BIG_ENDIAN                          = 0,
};

//status elements
enum {
  STATUS_ISSYNC                       = 0,
  STATUS_ID                           = 1,
  STATUS_DAGRANK                      = 2,
  STATUS_OUTBUFFERINDEXES             = 3,
  STATUS_ASN                          = 4,
  STATUS_MACSTATS                     = 5,
  STATUS_SCHEDULE                     = 6,
  STATUS_QUEUE                        = 7,
  STATUS_NEIGHBORS                    = 8,
  STATUS_MAX                          = 9,
};

// component identifiers
// the order is important because
enum {
  COMPONENT_NULL                      = 0x00,
  // Cross-layers
  COMPONENT_IDMANAGER                 = 0x01,
  COMPONENT_QUEUE                     = 0x02,
  COMPONENT_SERIAL                    = 0x03,
  COMPONENT_PACKET                    = 0x04,
  COMPONENT_RANDOM                    = 0x05,
  // PHY
  COMPONENT_RADIO                     = 0x06,
  // MAClow
  COMPONENT_IEEE802154                = 0x07,
  COMPONENT_IEEE802154E               = 0x08,

  // All components with higher component id than COMPONENT_IEEE802154E
  // won't be able to get free packets from the queue
  // when the node is not synch

  // MAClow<->MAChigh ("virtual components")
  COMPONENT_RES_TO_IEEE802154E        = 0x09,
  COMPONENT_IEEE802154E_TO_RES        = 0x0a,
  // MAChigh
  COMPONENT_RES                       = 0x0b,
  COMPONENT_NEIGHBORS                 = 0x0c,
  COMPONENT_TSCH                      = 0x0d,
  // Network
  COMPONENT_NETWORK                   = 0x0e,
  COMPONENT_FORWARD                   = 0x0f,
  COMPONENT_NOTIFICATION              = 0x10,

  // for aggregate
  COMPONENT_AGGREGATE                 = 0x11,
  COMPONENT_AGGREGATE_TO_IEEE802154E  = 0x12,

  // Application
  COMPONENT_CAN                       = 0x13,
  COMPONENT_SHT                       = 0x14,
  COMPONENT_TSL                       = 0x15,
  COMPONENT_UART					  = 0x20,
  COMPONENT_TRACE                     = 0x18,
  COMPONENT_ADV                       = 0x16,
  COMPONENT_ADV_TO_IEEE802154E        = 0x17,
};

/**
\brief error codes used throughout the IWSN stack

\note The comments are used in the Python parsing tool; {0} refers to the value of the
first argument, {1} refers to the second.
*/

enum {
  // Cross-layers

  // layer-1

  // layer-2a

  // layer-2b

  // layer-3

  // layer-4

  // layer-7

  // General

  // layer-7
  ERR_RCVD_ECHO_REQUEST               = 0x01, // received an echo request
  ERR_RCVD_ECHO_REPLY                 = 0x02, // received an echo reply
  ERR_GETDATA_ASKS_TOO_FEW_BYTES      = 0x03, // getData asks for too few bytes, maxNumBytes={0}, fill level={1}
  ERR_INPUT_BUFFER_OVERFLOW           = 0x04, // the input buffer has overflown
  // layer-4
  ERR_WRONG_TRAN_PROTOCOL             = 0x05, // unknown transport protocol {0} (code position {1))
  ERR_WRONG_TCP_STATE                 = 0x06, // wrong TCP state {0} (code position {1))
  ERR_RESET                           = 0x07, // TCP reset while in state {0} (code position {1))
  ERR_UNSUPPORTED_PORT_NUMBER         = 0x08, // unsupported port number {0} (code position {1))
  // layer-3
  ERR_UNSUPPORTED_ICMPV6_TYPE         = 0x09, // unsupported ICMPv6 type {0} (code position {1))
  ERR_6LOWPAN_UNSUPPORTED             = 0x0a, // unsupported 6LoWPAN parameter {1} at location {0}
  ERR_NO_NEXTHOP                      = 0x0b, // no next hop
  // layer-2b
  ERR_NEIGHBORS_FULL                  = 0x0c, // neighbors table is full (max number of neighbor is {0})
  ERR_NO_SENT_PACKET                  = 0x0d, // there is no sent packet in queue
  ERR_NO_RECEIVED_PACKET              = 0x0e, // there is no received packet in queue
  // layer-2a
  ERR_WRONG_CELLTYPE                  = 0x0f, // wrong celltype {0} at slotOffset {1}
  ERR_IEEE154_UNSUPPORTED             = 0x10, // unsupported IEEE802.15.4 parameter {1} at location {0}
  ERR_DESYNCHRONIZED                  = 0x11, // got desynchronized at slotOffset {0}
  ERR_SYNCHRONIZED                    = 0x12, // synchronized at slotOffset {0}
  ERR_WRONG_STATE_IN_ENDFRAME_SYNC    = 0x13, // wrong state {0} in end of frame+sync
  ERR_WRONG_STATE_IN_STARTSLOT        = 0x14, // wrong state {0} in startSlot, at slotOffset {1}
  ERR_WRONG_STATE_IN_TIMERFIRES       = 0x15, // wrong state {0} in timer fires, at slotOffset {1}
  ERR_WRONG_STATE_IN_NEWSLOT          = 0x16, // wrong state {0} in start of frame, at slotOffset {1}
  ERR_WRONG_STATE_IN_ENDOFFRAME       = 0x17, // wrong state {0} in end of frame, at slotOffset {1}
  ERR_MAXTXDATAPREPARE_OVERFLOW       = 0x18, // maxTxDataPrepare overflows while at state {0} in slotOffset {1}
  ERR_MAXRXACKPREPARE_OVERFLOWS       = 0x19, // maxRxAckPrepapare overflows while at state {0} in slotOffset {1}
  ERR_MAXRXDATAPREPARE_OVERFLOWS      = 0x1a, // maxRxDataPrepapre overflows while at state {0} in slotOffset {1}
  ERR_MAXTXACKPREPARE_OVERFLOWS       = 0x1b, // maxTxAckPrepapre overflows while at state {0} in slotOffset {1}
  ERR_WDDATADURATION_OVERFLOWS        = 0x1c, // wdDataDuration overflows while at state {0} in slotOffset {1}
  ERR_WDRADIO_OVERFLOWS               = 0x1d, // wdRadio overflows while at state {0} in slotOffset {1}
  ERR_WDRADIOTX_OVERFLOWS             = 0x1e, // wdRadioTx overflows while at state {0} in slotOffset {1}
  ERR_WDACKDURATION_OVERFLOWS         = 0x1f, // wdAckDuration overflows while at state {0} in slotOffset {1}
  // general
  ERR_BUSY_SENDING                    = 0x20, // busy sending
  ERR_UNEXPECTED_SENDDONE             = 0x21, // sendDone for packet I didn't send
  ERR_NO_FREE_PACKET_BUFFER           = 0x22, // no free packet buffer (code location {0})
  ERR_FREEING_UNUSED                  = 0x23, // freeing unused memory
  ERR_FREEING_ERROR                   = 0x24, // freeing memory unsupported memory
  ERR_UNSUPPORTED_COMMAND             = 0x25, // unsupported command {0}
  ERR_MSG_UNKNOWN_TYPE                = 0x26, // unknown message type {0}
  ERR_WRONG_ADDR_TYPE                 = 0x27, // wrong address type {0} (code location {1})
  ERR_BRIDGE_MISMATCH                 = 0x28, // isBridge mismatch (code location {0})
  ERR_HEADER_TOO_LONG                 = 0x29, // header too long, length {1} (code location {0})
  ERR_INPUTBUFFER_LENGTH              = 0x2a, // input length problem, length={0}

  ERR_WRONG_NEIGHBOR_PROPERTY         = 0x2b, // unknown neighbor property
  ERR_TSCH_FULL                       = 0x2c, // tsch buffer is full (max number of tsch buffer is {0})
  ERR_NO_TSCH_ENTRY                   = 0x2d, // tsch buffer has no the assigned slot
};

//=========================== typedef =========================================

typedef uint16_t  errorparameter_t;
typedef uint8_t   error_t;

#pragma pack(1)
typedef struct {
  uint8_t  byte4;
  uint16_t bytes2and3;
  uint16_t bytes0and1;
}__attribute__ ((packed)) asn_t;
#pragma pack()

#pragma pack(1)
typedef struct {                   // always written big endian, i.e. MSB in addr[0]
  iwsn_addr_type_t type;
  union {
    uint8_t addr_16b[2];
    uint8_t panid[2];
  };
}__attribute__ ((packed)) iwsn_addr_t;
#pragma pack()

//=========================== variables =======================================

//=========================== prototypes ======================================

void              iwsn_init();
void              iwsn_start();

PORT_TIMER_WIDTH  iwsn_asnDiff(asn_t* oldASN, asn_t* newASN);

#endif /* IWSN_H_ */
