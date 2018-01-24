/**
\brief General IWSN 02aMAClow-layer 'ieee802154' definitions

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "string.h"
#include "ieee802154.h"
#include "packet.h"
#include "idmanager.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================

void ieee802154_prependHeader(QueueEntry_t*     msg,
                              uint8_t           frameType,
                              bool              securityEnabled,
                              uint8_t           sequenceNumber,
                              iwsn_addr_t*      nextHop)
{
  uint8_t temp_8b;
  
  // DLPDU specifier
  packet_reserveHeaderSize(msg, sizeof(uint8_t));
  *((uint8_t*)(msg->payload)) = msg->l2_pduSpecifier;

  // poipoi: using 16-bit source address
  packet_writeAddress(msg, idmanager_getMyID(ADDR_16B), LITTLE_ENDIAN);
  // nextHop address
  if (packet_isBroadcastMulticast(nextHop))
  {
    //broadcast address is always 16-bit
    packet_reserveHeaderSize(msg, sizeof(uint8_t));
    *((uint8_t*)(msg->payload)) = 0xFF;
    packet_reserveHeaderSize(msg, sizeof(uint8_t));
    *((uint8_t*)(msg->payload)) = 0xFF;
  }
  else
  {
    // poipoi: using 16-bit destination address
    //packet_reserveHeaderSize(msg, sizeof(uint8_t));
    //*((uint8_t*)(msg->payload)) = nextHop->addr_16b[0];
    //packet_reserveHeaderSize(msg, sizeof(uint8_t));
    //*((uint8_t*)(msg->payload)) = nextHop->addr_16b[1];
    
    switch (nextHop->type)
    {
      case ADDR_16B:
        packet_writeAddress(msg, nextHop, LITTLE_ENDIAN);
        break;
      default:
        break;
    }
  }
  // destpan
  packet_writeAddress(msg, idmanager_getMyID(ADDR_PANID), LITTLE_ENDIAN);
  // dsn
  packet_reserveHeaderSize(msg, sizeof(uint8_t));
  *((uint8_t*)(msg->payload)) = sequenceNumber;
  // fcf (2nd byte)
  temp_8b              = 0;
  // poipoi: 16-bit source/dest addresses
  temp_8b             |= IEEE154_ADDR_SHORT              << IEEE154_FCF_DEST_ADDR_MODE;
  temp_8b             |= IEEE154_ADDR_SHORT              << IEEE154_FCF_SRC_ADDR_MODE;
  
  /*
  if (packet_isBroadcastMulticast(nextHop))
  {
    temp_8b           |= IEEE154_ADDR_SHORT              << IEEE154_FCF_DEST_ADDR_MODE;
  } 
  else 
  {
    switch (nextHop->type)
    {
      case ADDR_16B:
        temp_8b       |= IEEE154_ADDR_SHORT              << IEEE154_FCF_DEST_ADDR_MODE;
        break;
      default:
        break;
    }
  }
  temp_8b             |= IEEE154_ADDR_EXT                << IEEE154_FCF_SRC_ADDR_MODE;
  */
  
  packet_reserveHeaderSize(msg, sizeof(uint8_t));
  *((uint8_t*)(msg->payload)) = temp_8b;
  //fcf (1st byte)
  temp_8b              = 0;
  temp_8b             |= frameType                       << IEEE154_FCF_FRAME_TYPE;
  temp_8b             |= securityEnabled                 << IEEE154_FCF_SECURITY_ENABLED;
  temp_8b             |= IEEE154_PENDING_NO_FRAMEPENDING << IEEE154_FCF_FRAME_PENDING;
  if (frameType == IEEE154_TYPE_ACK || packet_isBroadcastMulticast(nextHop)) 
  {
    temp_8b           |= IEEE154_ACK_NO_ACK_REQ          << IEEE154_FCF_ACK_REQ;
  } 
  else 
  {
    temp_8b           |= IEEE154_ACK_YES_ACK_REQ         << IEEE154_FCF_ACK_REQ;
  }
  temp_8b             |= IEEE154_PANID_COMPRESSED        << IEEE154_FCF_INTRAPAN;
  packet_reserveHeaderSize(msg,sizeof(uint8_t));
  *((uint8_t*)(msg->payload)) = temp_8b;
}

void ieee802154_retrieveHeader(QueueEntry_t*          msg,
                               ieee802154_header_iht* ieee802514_header)
{
  uint8_t temp_8b;
  
  ieee802514_header->valid = false;
  ieee802514_header->headerLength = 0;
  
  //fcf, byte 1
  if (ieee802514_header->headerLength > msg->length)
  {
    return;
  }
  temp_8b = *((uint8_t*)(msg->payload) + ieee802514_header->headerLength);
  ieee802514_header->frameType         = (temp_8b >> IEEE154_FCF_FRAME_TYPE      ) & 0x07;//3b
  ieee802514_header->securityEnabled   = (temp_8b >> IEEE154_FCF_SECURITY_ENABLED) & 0x01;//1b
  ieee802514_header->framePending      = (temp_8b >> IEEE154_FCF_FRAME_PENDING   ) & 0x01;//1b
  ieee802514_header->ackRequested      = (temp_8b >> IEEE154_FCF_ACK_REQ         ) & 0x01;//1b
  ieee802514_header->panIDCompression  = (temp_8b >> IEEE154_FCF_INTRAPAN        ) & 0x01;//1b
  ieee802514_header->headerLength     += 1;
  //fcf, byte 2
  if (ieee802514_header->headerLength > msg->length)
  {
    return;
  }
  temp_8b = *((uint8_t*)(msg->payload) + ieee802514_header->headerLength);
  switch ((temp_8b >> IEEE154_FCF_DEST_ADDR_MODE ) & 0x03 ) 
  {
    case IEEE154_ADDR_NONE:
      ieee802514_header->dest.type = ADDR_NONE;
      break;
    case IEEE154_ADDR_SHORT:
      ieee802514_header->dest.type = ADDR_16B;
      break;
    case IEEE154_ADDR_EXT:
      ieee802514_header->dest.type = ADDR_64B;
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_IEEE802154, ERR_IEEE154_UNSUPPORTED,
                       (errorparameter_t)1,
                       (errorparameter_t)(temp_8b >> IEEE154_FCF_DEST_ADDR_MODE) & 0x03)
#endif
      break;
  }
  switch ((temp_8b >> IEEE154_FCF_SRC_ADDR_MODE) & 0x03 ) 
  {
    case IEEE154_ADDR_NONE:
      ieee802514_header->src.type = ADDR_NONE;
      break;
    case IEEE154_ADDR_SHORT:
      ieee802514_header->src.type = ADDR_16B;
      break;
    case IEEE154_ADDR_EXT:
      ieee802514_header->src.type = ADDR_64B;
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_IEEE802154, ERR_IEEE154_UNSUPPORTED,
                       (errorparameter_t)2,
                       (errorparameter_t)(temp_8b >> IEEE154_FCF_SRC_ADDR_MODE) & 0x03);
#endif
      break;
  }
  ieee802514_header->headerLength += 1;
  //sequenceNumber
  if (ieee802514_header->headerLength > msg->length) 
  {
    return;
  }
  ieee802514_header->dsn  = *((uint8_t*)(msg->payload) + ieee802514_header->headerLength);
  ieee802514_header->headerLength += 1;
  //panID
  if (ieee802514_header->headerLength > msg->length) 
  {
    return;
  }
  packet_readAddress(((uint8_t*)(msg->payload) + ieee802514_header->headerLength),
                     ADDR_PANID,
                     &ieee802514_header->panid,
                     LITTLE_ENDIAN);
  ieee802514_header->headerLength += 2;
  //dest
  if (ieee802514_header->headerLength > msg->length) 
  {
    return;
  }
  switch (ieee802514_header->dest.type)
  {
    case ADDR_NONE:
      break;
    case ADDR_16B:
      packet_readAddress(((uint8_t*)(msg->payload) + ieee802514_header->headerLength),
                         ADDR_16B,
                         &ieee802514_header->dest,
                         LITTLE_ENDIAN);
      ieee802514_header->headerLength += 2;      
      
      if (ieee802514_header->headerLength > msg->length)
      {
        return;
      }
      break;
    default:
      break;
  }
  //src
  switch (ieee802514_header->src.type)
  {
    case ADDR_NONE:
      break;
    case ADDR_16B:
      packet_readAddress(((uint8_t*)(msg->payload) + ieee802514_header->headerLength),
                         ADDR_16B,
                         &ieee802514_header->src,
                         LITTLE_ENDIAN);
      ieee802514_header->headerLength += 2;
      
      if (ieee802514_header->headerLength > msg->length)
      {
        return;
      }
      break;
    default:
      break;
  }
  //DLPDU specifier
  ieee802514_header->pduSpecifier = *((uint8_t*)(msg->payload) + ieee802514_header->headerLength);
  ieee802514_header->headerLength += 1;
  // if you reach this, the header is valid
  ieee802514_header->valid = true;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
