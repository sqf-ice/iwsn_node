/**
\brief General IWSN Cross-layer 'packet' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __PACKET_H
#define __PACKET_H

//=========================== include =========================================

#include "queue.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

// address recognition
bool     packet_isBroadcastMulticast(iwsn_addr_t* address);

// read/write addresses to/from packets
void     packet_readAddress(uint8_t* payload, uint8_t type, iwsn_addr_t* writeToAddress, bool littleEndian);
void     packet_writeAddress(QueueEntry_t* msg, iwsn_addr_t* address, bool littleEndian);

// reserving/tossing headers and footers
void     packet_reserveHeaderSize(QueueEntry_t* pkt, uint8_t header_length);
void     packet_tossHeader(QueueEntry_t* pkt, uint8_t header_length);
void     packet_reserveFooterSize(QueueEntry_t* pkt, uint8_t header_length);
void     packet_tossFooter(QueueEntry_t* pkt, uint8_t header_length);

// calculate CRC
void     packet_calculateCRC(QueueEntry_t* msg);
bool     packet_checkCRC(QueueEntry_t* msg);

// calculate checksum
void     packet_calculateChecksum(QueueEntry_t* msg, uint8_t* checksum_ptr);

#endif
