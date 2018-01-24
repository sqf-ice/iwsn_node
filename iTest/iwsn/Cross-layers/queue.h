/**
\brief General IWSN Cross-layer 'queue' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __QUEUE_H
#define __QUEUE_H

//=========================== include =========================================

#include "stdint.h"
#include "iwsn.h"
#include "stdbool.h"

//=========================== define ==========================================

#define QUEUE_LENGTH  30

//=========================== typedef =========================================

typedef struct {
  // ===== admin
  uint8_t       creator;                        // the component which called getFreePacketBuffer()
  uint8_t       owner;                          // the component which currently owns the entry
  uint8_t*      payload;                        // pointer to the start of the payload within 'packet'
  uint8_t       length;                         // length in bytes of the payload
  
  asn_t         asn;                            // the absolute slot number when the pkt is created
  
  uint8_t       ag_length;                      // for aggregate
  uint8_t*      aggregate;                      // for aggregate

  // layer-7
  
  // layer-4
  
  // layer-3
  iwsn_addr_t   l3_destinationORsource;         // network destination (down stack) or source address (up)
  uint8_t       l3_pduSpecifier;                // network layer PDU specifier of the packet
  uint8_t*		l3_payload; 					// pointer to the start of the payload of l3 (used for MAC to fill in info)

  // layer-2
  error_t       l2_sendDoneError;               // outcome of trying to send this packet
  iwsn_addr_t   l2_nextORpreviousHop;           // 64b IEEE802.15.4 next (down stack) or previous (up) hop address
  uint8_t       l2_frameType;                   // beacon, data, ack, cmd
  uint8_t       l2_dsn;                         // sequence number of the received frame
  bool          l2_shared;                      // whether or not the packet needs to be sent in the shared slot
  uint8_t       l2_backoffExponent;             
  uint8_t       l2_backoff;                      
  uint8_t       l2_retriesLeft;                 // number Tx retries left before packet dropped (dropped when hits 0)
  uint8_t       l2_numTxAttempts;               // number Tx attempts
  asn_t         l2_asn;                         // at what ASN the packet was Tx'ed or Rx'ed
  uint8_t       l2_pduSpecifier;                // datalink layer PDU specifier of the frame
  uint8_t*      l2_payload;                     // pointer to the start of the payload of l2 (used for MAC to fill in ASN in ADV)
  
  // layer-1
  uint8_t       l1_txPower;                     // power for packet to Tx at
  int8_t        l1_rssi;                        // RSSI of received packet
  uint8_t       l1_lqi;                         // LQI of received packet
  uint8_t       l1_crc;                         // did received packet pass CRC check?
  
  //the packet content
  uint8_t       packet[1+1+125+2+1];            // 1B spi address, 1B length, 125B data, 2B CRC, 1B LQI
} QueueEntry_t;

typedef struct {
  QueueEntry_t queue[QUEUE_LENGTH];
  uint8_t available_length;             //add by dong for busy
} queue_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void          queue_init();
QueueEntry_t* queue_getFreePacketBuffer(uint8_t creator);
error_t       queue_freePacketBuffer(QueueEntry_t* pkt);

uint8_t       queue_getAvailableLength();

QueueEntry_t* queue_resGetSentPacket();
QueueEntry_t* queue_resGetReceivedPacket();

QueueEntry_t* queue_macGetAdvPacket();
QueueEntry_t* queue_macGetSyncPacket();
QueueEntry_t* queue_macGetSharedPacket(iwsn_addr_t* toNeighbor, uint8_t netType);
QueueEntry_t* queue_macGetDedicatedPacket(iwsn_addr_t* toNeighbor, uint8_t pktType);

QueueEntry_t* queue_getAggregatePacket(iwsn_addr_t* toNeighbor, uint8_t length);
QueueEntry_t* queue_getTimeoutPacket();

void          queue_macIndicateTx(QueueEntry_t* pkt, bool succesfullTx);

#endif
