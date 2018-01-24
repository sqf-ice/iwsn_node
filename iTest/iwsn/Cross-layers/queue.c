/**
\brief General IWSN Cross-layer 'queue' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================
#ifdef HAVE_CONFIG
#include "config.h"   //this is set to the HAVE_ADV_ADDITIONAL and HAVE_AGGREGATE
#endif  

#include "string.h"
#include "queue.h"
#include "packet.h"
#include "idmanager.h"
#include "random.h"
#include "ieee802154.h"
#include "ieee802154e.h"
#include "res.h"
#include "aggregate.h"
#include "monitor.h"	//v2
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

queue_vars_t queue_vars;

//=========================== prototypes ======================================

void queue_reset_entry(QueueEntry_t* entry);

//=========================== main ============================================

//=========================== public ==========================================

// ====== admin
void queue_init()
{
  uint8_t i;
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    queue_reset_entry(&(queue_vars.queue[i]));
  }
  
  queue_vars.available_length        = QUEUE_LENGTH;
}

// ====== called by any component
QueueEntry_t* queue_getFreePacketBuffer(uint8_t creator)
{
  uint8_t i;
  asn_t *curAsn;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  // Enable only allocation of free buffers to MAC layer in case we are not sync
  if (ieee802154e_isSynch() == false && creator > COMPONENT_IEEE802154E)
  {
    ENABLE_INTERRUPTS();
    return NULL;
  }
  
  // In synch or mac layer request
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if (queue_vars.queue[i].owner == COMPONENT_NULL)
    {
      curAsn = ieee802154e_getCurASN();
      queue_vars.queue[i].owner = COMPONENT_QUEUE;
      queue_vars.queue[i].creator = creator;
      queue_vars.queue[i].asn.bytes0and1 = curAsn->bytes0and1;
      queue_vars.queue[i].asn.bytes2and3 = curAsn->bytes2and3;
      queue_vars.queue[i].asn.byte4      = curAsn->byte4;
      queue_vars.available_length--;   // add by dong for busy
      ENABLE_INTERRUPTS(); 
      return &queue_vars.queue[i];
    }
  }
  ENABLE_INTERRUPTS();
  return NULL;
}

error_t queue_freePacketBuffer(QueueEntry_t* pkt)
{
  uint8_t i;  
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if ((&queue_vars.queue[i] == pkt) &&
        (queue_vars.queue[i].owner != COMPONENT_NULL) &&
        (queue_vars.queue[i].creator != COMPONENT_NULL))
    {           
      queue_reset_entry(&(queue_vars.queue[i]));
      queue_vars.available_length++;   // add by dong for busy
      ENABLE_INTERRUPTS();
      return E_SUCCESS;
    }
  }
  
  ENABLE_INTERRUPTS();
  return E_FAIL;
}

uint8_t queue_getAvailableLength()
{
  uint8_t res;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  res = queue_vars.available_length;
  ENABLE_INTERRUPTS();
  return res;
}

//======= called by RES

QueueEntry_t* queue_resGetSentPacket()
{
  uint8_t i;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if (queue_vars.queue[i].owner == COMPONENT_IEEE802154E_TO_RES && 
        queue_vars.queue[i].creator != COMPONENT_IEEE802154E)
    {           
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }
  
  ENABLE_INTERRUPTS();
  return NULL;
}

QueueEntry_t* queue_resGetReceivedPacket()
{
  uint8_t i;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if (queue_vars.queue[i].owner == COMPONENT_IEEE802154E_TO_RES && 
        queue_vars.queue[i].creator == COMPONENT_IEEE802154E)
    {           
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }
  
  ENABLE_INTERRUPTS();
  return NULL;
}

//======= called by IEEE80215E

QueueEntry_t* queue_macGetAdvPacket()
{
  uint8_t i;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if ((queue_vars.queue[i].owner == COMPONENT_RES_TO_IEEE802154E
#ifdef HAVE_ADV_ADDITIONAL
         || queue_vars.queue[i].owner == COMPONENT_ADV_TO_IEEE802154E ||
         queue_vars.queue[i].owner == COMPONENT_ADV
#endif
         ) &&
        queue_vars.queue[i].creator == COMPONENT_RES && 
        queue_vars.queue[i].l2_frameType == IEEE154_TYPE_DATA &&
        ((queue_vars.queue[i].l2_pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ADV)  &&
        packet_isBroadcastMulticast(&(queue_vars.queue[i].l2_nextORpreviousHop)))
    {
#ifdef HAVE_ADV_ADDITIONAL
      if (queue_vars.queue[i].owner == COMPONENT_ADV)
      {
        adv_add_send_internal(&queue_vars.queue[i]);
      }
#endif
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }

  ENABLE_INTERRUPTS();
  return NULL;
}

QueueEntry_t* queue_macGetSyncPacket()
{
  uint8_t i;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();

  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if (queue_vars.queue[i].owner == COMPONENT_RES_TO_IEEE802154E &&
        queue_vars.queue[i].creator == COMPONENT_RES &&
        queue_vars.queue[i].l2_frameType == IEEE154_TYPE_DATA &&
        ((queue_vars.queue[i].l2_pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_SYNC)  &&
        packet_isBroadcastMulticast(&(queue_vars.queue[i].l2_nextORpreviousHop)))
    {
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }
  
  ENABLE_INTERRUPTS();
  return NULL;
}

// return a packet that needs to be sent on the shared slot
QueueEntry_t* queue_macGetSharedPacket(iwsn_addr_t* toNeighbor,uint8_t netType)
{
  uint8_t i;
  QueueEntry_t* ret = NULL;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if ((
#ifdef HAVE_AGGREGATE
         queue_vars.queue[i].owner == COMPONENT_AGGREGATE_TO_IEEE802154E ||
         queue_vars.queue[i].owner == COMPONENT_AGGREGATE ||
#endif
        queue_vars.queue[i].owner == COMPONENT_RES_TO_IEEE802154E) &&
        queue_vars.queue[i].l2_frameType == IEEE154_TYPE_DATA &&
        queue_vars.queue[i].l2_shared == true  &&
        queue_vars.queue[i].l2_backoff == 0 &&
        queue_vars.queue[i].l3_payload[0] == netType &&
        ((toNeighbor->type == ADDR_NONE) || (idmanager_isSameAddress(toNeighbor, &queue_vars.queue[i].l2_nextORpreviousHop) == true)) &&
        packet_isBroadcastMulticast(&(queue_vars.queue[i].l2_nextORpreviousHop)) == false)
    {
#ifdef HAVE_AGGREGATE
      if (queue_vars.queue[i].owner == COMPONENT_AGGREGATE)
      {
        aggregate_send_internal(&queue_vars.queue[i]);
      }
#endif
      ret = &queue_vars.queue[i];
    }
    else if (queue_vars.queue[i].l2_backoff > 0)
    {
      queue_vars.queue[i].l2_backoff--;
    }
  }
  
  ENABLE_INTERRUPTS();
  return ret;
}

// return a packet that needs to be sent on the dedicated slot
QueueEntry_t* queue_macGetDedicatedPacket(iwsn_addr_t* toNeighbor, uint8_t pktType)
{
  uint8_t i;
  uint8_t pktType2 = 0xFF;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  if(pktType == NET_TYPE_CAN_SAMPLE)
  {
	  pktType2 = NET_TYPE_DATA_AGGREGATE;
  }
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if ((
#ifdef HAVE_AGGREGATE
         queue_vars.queue[i].owner == COMPONENT_AGGREGATE_TO_IEEE802154E ||
         queue_vars.queue[i].owner == COMPONENT_AGGREGATE ||
#endif
         queue_vars.queue[i].owner == COMPONENT_RES_TO_IEEE802154E) &&
        queue_vars.queue[i].l2_frameType == IEEE154_TYPE_DATA &&
        queue_vars.queue[i].l2_shared == false  &&
        (queue_vars.queue[i].l3_pduSpecifier == pktType ||
        		queue_vars.queue[i].l3_pduSpecifier == pktType2)  &&
        idmanager_isSameAddress(toNeighbor, &queue_vars.queue[i].l2_nextORpreviousHop) == true &&
        packet_isBroadcastMulticast(&(queue_vars.queue[i].l2_nextORpreviousHop)) == false)
    {
#ifdef HAVE_AGGREGATE
      if (queue_vars.queue[i].owner == COMPONENT_AGGREGATE)
      {
        aggregate_send_internal(&queue_vars.queue[i]);
      }
#endif
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }
  
  ENABLE_INTERRUPTS();
  return NULL;
}

#ifdef HAVE_AGGREGATE
QueueEntry_t* queue_getAggregatePacket(iwsn_addr_t* toNeighbor, uint8_t length)
{
  uint8_t i;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();

  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if (queue_vars.queue[i].owner == COMPONENT_AGGREGATE &&
        queue_vars.queue[i].l2_frameType == IEEE154_TYPE_DATA &&
        idmanager_isSameAddress(toNeighbor, &queue_vars.queue[i].l3_destinationORsource) == true &&
        aggregate_getLeftLength(&queue_vars.queue[i]) >= length)
    {
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }

  ENABLE_INTERRUPTS();
  return NULL;
}
#endif

QueueEntry_t* queue_getTimeoutPacket()
{
  uint8_t i;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  for (i = 0; i < QUEUE_LENGTH; i++)
  {
    if ((queue_vars.queue[i].creator != COMPONENT_NULL ||
         queue_vars.queue[i].owner != COMPONENT_NULL) &&
        (iwsn_asnDiff(&queue_vars.queue[i].asn, ieee802154e_getCurASN()) >= 1500))
    {           
      ENABLE_INTERRUPTS();
      return &queue_vars.queue[i];
    }
  }
  
  ENABLE_INTERRUPTS();
  return NULL;
}

void queue_macIndicateTx(QueueEntry_t* pkt, bool succesfullTx)
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
	if (succesfullTx == true)
	{
		// reset backoffExponent
		pkt->l2_backoffExponent   = MINBE;
		// reset backoff
		pkt->l2_backoff           = 0;
	}
	else
	{
		// increase the backoffExponent
		if (pkt->l2_backoffExponent < MAXBE)
		{
		  pkt->l2_backoffExponent++;
		}
		// set the backoff to a random value in [0..2^BE]
		pkt->l2_backoff = random_get16b() % (1 << pkt->l2_backoffExponent);
	}
    ENABLE_INTERRUPTS();
}

//=========================== private =========================================
void queue_reset_entry(QueueEntry_t* entry)
{
  //admin
  entry->creator                     = COMPONENT_NULL;
  entry->owner                       = COMPONENT_NULL;
  entry->payload                     = &(entry->packet[127]);
  entry->length                      = 0;
  
  //l7
  
  //l4
  
  //l3
  entry->l3_pduSpecifier             = 0;
  //l2
  entry->l2_nextORpreviousHop.type   = ADDR_NONE;
  entry->l2_frameType                = IEEE154_TYPE_UNDEFINED;
  entry->l2_pduSpecifier             = 0;
  entry->l2_retriesLeft              = 0;
  entry->l2_shared                   = false;         //add by dong for shared
  entry->l2_backoffExponent          = MINBE;         //add by dong for shared
  entry->l2_backoff                  = 0;             //add by dong for shared
}
//=========================== interrupt handlers ==============================


