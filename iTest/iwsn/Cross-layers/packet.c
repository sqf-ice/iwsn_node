/**
\brief General IWSN Cross-layer 'packet' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "queue.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void onesComplementSum(uint8_t* global_sum, uint8_t* ptr, int length);

//=========================== main ============================================

//=========================== public ==========================================

// ====== address recognition
bool packet_isBroadcastMulticast(iwsn_addr_t* address)
{
  uint8_t i;
  uint8_t address_length;
  
  // 15.4 broadcast
  switch (address->type) 
  {
    case ADDR_16B:
      address_length = 2;
      break;
    default:
      return false;
  }
  for (i = 0; i < address_length; i++) 
  {
    // add by dong for 128B address
    if (address->addr_16b[i] != 0xFF)
    {
      return false;
    }
  }
  return true;
}

//======= address read/write
void packet_readAddress(uint8_t* payload, uint8_t type, iwsn_addr_t* writeToAddress, bool littleEndian) 
{
  uint8_t i;
  uint8_t address_length;
  
  writeToAddress->type = (iwsn_addr_type_t)type;
  switch (type)
  {
    case ADDR_16B:
    case ADDR_PANID:
      address_length = 2;
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_PACKETFUNCTIONS, ERR_WRONG_ADDR_TYPE,
                       (errorparameter_t)type,
                       (errorparameter_t)6);
#endif
      return;
  }
  for (i = 0; i < address_length; i++)
  {
    if (littleEndian) 
    {
      // add by dong for 128B address
      writeToAddress->addr_16b[address_length - 1 - i] = *(payload + i);
    } 
    else 
    {
      // add by dong for 128B address
      writeToAddress->addr_16b[i]   = *(payload + i);
    }
  }
}

void packet_writeAddress(QueueEntry_t* msg, iwsn_addr_t* address, bool littleEndian)
{
  uint8_t i;
  uint8_t address_length;
  
  switch (address->type)
  {
    case ADDR_16B:
    case ADDR_PANID:
      address_length = 2;
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_PACKETFUNCTIONS, ERR_WRONG_ADDR_TYPE,
                       (errorparameter_t)address->type,
                       (errorparameter_t)7);
#endif
      return;
  }
  for (i = 0; i < address_length; i++) 
  {
    msg->payload      -= sizeof(uint8_t);
    msg->length       += sizeof(uint8_t);
    if (littleEndian) 
    {
    //add by dong for 128B address
    *((uint8_t*)(msg->payload)) = address->addr_16b[i];
    } 
    else 
    {
      //add by dong for 128B address
      *((uint8_t*)(msg->payload)) = address->addr_16b[address_length - 1 - i];
    }
  }
}

// ====== reserving/tossing headers
void packet_reserveHeaderSize(QueueEntry_t* pkt, uint8_t header_length) 
{
  pkt->payload -= header_length;
  pkt->length  += header_length;
#ifdef HAVE_DEBUG
  if ((uint8_t*)(pkt->payload) < (uint8_t*)(pkt->packet))
  {
    debug_printError(COMPONENT_PACKETFUNCTIONS, ERR_HEADER_TOO_LONG,
                     (errorparameter_t)0,
                     (errorparameter_t)pkt->length);
  }
#endif
}

void packet_tossHeader(QueueEntry_t* pkt, uint8_t header_length) 
{
  pkt->payload += header_length;
  pkt->length  -= header_length;
}

void packet_reserveFooterSize(QueueEntry_t* pkt, uint8_t header_length) 
{
  pkt->length  += header_length;
}
void packet_tossFooter(QueueEntry_t* pkt, uint8_t header_length) 
{
  pkt->length  -= header_length;
}

// ====== CRC calculation
void packet_calculateCRC(QueueEntry_t* msg) 
{
  uint16_t crc;
  uint8_t  i;
  uint8_t  count;

  crc = 0;
  for (count = 1; count < msg->length - 2; count++) 
  {
    crc = crc ^ (uint8_t)*(msg->payload + count);
    //crc = crc ^ (uint16_t)*ptr++ << 8;
    for (i = 0; i < 8; i++) 
    {
      if (crc & 0x1) 
      {
        crc = crc >> 1 ^ 0x8408;
      } 
      else 
      {
        crc = crc >> 1;
      }
    }
  }
  *(msg->payload + (msg->length - 2)) = crc % 256;
  *(msg->payload + (msg->length - 1)) = crc / 256;
}

bool packet_checkCRC(QueueEntry_t* msg) 
{
  uint16_t crc;
  uint8_t  i;
  uint8_t  count;

  crc = 0;
  for (count = 0; count < msg->length - 2; count++) 
  {
    crc = crc ^ (uint8_t)*(msg->payload + count);
    //crc = crc ^ (uint16_t)*ptr++ << 8;
    for (i = 0; i < 8; i++) 
    {
      if (crc & 0x1) 
      {
        crc = crc >> 1 ^ 0x8408;
      } 
      else 
      {
        crc = crc >> 1;
      }
    }
  }
  
  if (*(msg->payload + (msg->length - 2)) == crc % 256 
      && *(msg->payload + (msg->length - 1)) == crc / 256) 
  {
    return true;
  } 
  else 
  {
    return false;
  }
}

//=========================== private =========================================

void onesComplementSum(uint8_t* global_sum, uint8_t* ptr, int length) 
{
  uint32_t sum = 0xFFFF & (global_sum[0]<<8 | global_sum[1]);
  
  while (length > 1) 
  {
    sum     += 0xFFFF & (*ptr<<8 | *(ptr+1));
    ptr     += 2;
    length  -= 2;
  }
  
  if (length) 
  {
    sum     += (0xFF & *ptr)<<8;
  }
  
  while ( sum >> 16)
  {
    sum      = (sum & 0xFFFF)+(sum >> 16);
  }
  
  global_sum[0] = (sum>>8) & 0xFF;
  global_sum[1] = sum & 0xFF;
}

//=========================== interrupt handlers ==============================
