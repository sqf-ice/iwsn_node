/**
\brief General IWSN 03-Network 'forward' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================
#ifdef HAVE_CONFIG
#include "config.h"    //this is set to the HAVE_AGGREGATE
#endif

#include "string.h"
#include "forward.h"
#include "idmanager.h"
#include "packet.h"
#include "res.h"
#include "ieee802154.h"
#include "monitor.h"
//#include "can_trx.h"
#include "aggregate.h"
#include "neighbors.h"
//#include "applications.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================

void forward_receive(QueueEntry_t* msg, net_header_iht net_header)
{
  QueueEntry_t*   pkt;
  uint8_t         type    = msg->payload[0];
  uint8_t         subType = msg->payload[1];
  uint8_t         myRole  = idmanager_getMyRole();
  uint8_t         temp_8b;
  iwsn_addr_t     tempAddr;
  tempAddr.type = ADDR_NONE;

#ifdef HAVE_AGGREGATE  
  if (type == NET_TYPE_DATA_AGGREGATE)
  {
    if (subType == AGGREGATE_CAN_SAMPLE_COLLECT)
    {
      //if ((pkt = queue_getAggregatePacket(&net_header.dst, msg->payload[2] * sizeof(canAggrEntry_t))) != NULL)
      if ((pkt = queue_getAggregatePacket(&net_header.dst, msg->payload[2])) != NULL)

      {
        // declare ownership over that packet
        pkt->owner    = COMPONENT_CAN;
    
        // append the can message to the packet
       // aggregate_appendData(pkt, &msg->payload[3], msg->payload[2] * sizeof(canAggrEntry_t), msg->payload[2]);  // can aggregate block
        aggregate_appendData(pkt, &msg->payload[3], msg->payload[7], 1);  // can aggregate block
        queue_freePacketBuffer(msg);
        return;
      }
    }
    else
    {
      queue_freePacketBuffer(msg);
      return;
    }

    // get a free queue buffer for forwarding the message
    if (queue_getAvailableLength() <= 2 ||
        (pkt = queue_getFreePacketBuffer(COMPONENT_FORWARD)) == NULL)
    {
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_FORWARD, ERR_NO_FREE_PACKET_BUFFER,
                       (errorparameter_t)0,
                       (errorparameter_t)0);
#endif
      queue_freePacketBuffer(msg);
      return;
    }
    
    // decare ownership over that packet
    pkt->creator = COMPONENT_FORWARD;
    pkt->owner   = COMPONENT_FORWARD;
    
    if (subType == AGGREGATE_CAN_SAMPLE_COLLECT)
    {
      packet_reserveHeaderSize(pkt, MAX_LENGTH_AGGREGATE);
      
      pkt->aggregate = pkt->payload;
      pkt->ag_length = 0;
      
      memcpy(&pkt->payload[0], &msg->payload[0], (msg->length));
      pkt->ag_length += msg->length;
    }
    else
    {
      queue_freePacketBuffer(pkt);
      queue_freePacketBuffer(msg);
      return;
    }
    
    // some l3 information
    pkt->l3_pduSpecifier = type;
    memcpy(&pkt->l3_destinationORsource, &net_header.dst, sizeof(iwsn_addr_t));
    
    // some l2 information
    temp_8b  = 0;
    temp_8b |= (MAC_PDU_PKT_TYPE_DATA << MAC_PDU_PKT_TYPE)  & 0x07;
    temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;
    
    pkt->l2_frameType                     = IEEE154_TYPE_DATA;
    pkt->l2_pduSpecifier                  = temp_8b;
    pkt->l2_shared                        = false;
    
    // prepare net header
    net_prependNetHeaderForForward(pkt, net_header);
    
    // 
    if (tempAddr.type != ADDR_NONE)
    {
      memcpy(&pkt->l3_destinationORsource, &tempAddr, sizeof(iwsn_addr_t));
    }
    
    queue_freePacketBuffer(msg);
    net_forward(pkt, net_header);
    
    return;
  }
#endif
  
  // get a free queue buffer for forwarding the message
  if (queue_getAvailableLength() <= 2 ||
      (pkt = queue_getFreePacketBuffer(COMPONENT_FORWARD)) == NULL)
  {
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_FORWARD, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    queue_freePacketBuffer(msg);
    return;
  }
  
  // decare ownership over that packet
  pkt->creator = COMPONENT_FORWARD;
  pkt->owner   = COMPONENT_FORWARD;
  
  // reserve space for the message content
  if (type == NET_TYPE_NODE_NOTIFY)
  {
    if (subType == SLOT_COLLISION_NOTIFY)
    {
      packet_reserveHeaderSize(pkt, (msg->length));
      memcpy(&pkt->payload[0], &msg->payload[0], (msg->length));
    } else{
      queue_freePacketBuffer(pkt);
      queue_freePacketBuffer(msg);
      return;
    }
  }
  else if (type == NET_TYPE_CAN_SAMPLE)
  {
/*    switch (subType)
    {
      case CAN_SAMPLE_COLLECT:
        packet_reserveHeaderSize(pkt, (msg->length));
        memcpy(&pkt->payload[0], &msg->payload[0], (msg->length));
		
        break;
      default:
        queue_freePacketBuffer(pkt);
        queue_freePacketBuffer(msg);
        return;
    }*/
  }
  else if (type == NET_TYPE_CMD_CONTROL)
  {
    switch (subType)
    {
      case CMD_CONTROL_UNICAST:
    	  if(idmanager_isGateway(myRole) == true)
    	  {
			packet_reserveHeaderSize(pkt, (msg->length - 2));
			pkt->payload[0] = type;
			pkt->payload[1] = subType;
			pkt->payload[2] = 0;
			memcpy(&pkt->payload[3], &msg->payload[5], (msg->length - 5));
    	  }else{
    		packet_reserveHeaderSize(pkt, (msg->length));
    		memcpy(&pkt->payload[0], &msg->payload[0], (msg->length));
    	  }
        break;
      default:
        queue_freePacketBuffer(pkt);
        queue_freePacketBuffer(msg);
        return;
    }
  }
  else
  {
    queue_freePacketBuffer(pkt);
    queue_freePacketBuffer(msg);
    return;
  }
  
  // some l3 information
  pkt->l3_pduSpecifier = type;
  memcpy(&pkt->l3_destinationORsource, &net_header.dst, sizeof(iwsn_addr_t));
  
  // some l2 information
  temp_8b  = 0;
  temp_8b |= (MAC_PDU_PKT_TYPE_DATA << MAC_PDU_PKT_TYPE)  & 0x07;
  temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;
  
  pkt->l2_frameType                     = IEEE154_TYPE_DATA;
  pkt->l2_pduSpecifier                  = temp_8b;
  if ((type == NET_TYPE_NODE_NOTIFY) || (type == NET_TYPE_MONITOR_STATUS)
		  || (idmanager_isGateway(myRole) == true))
  {
	  pkt->l2_shared                        = true;  //v3
  }
  else
  {
	  pkt->l2_shared                        = false;   //v3
  }

  // prepare net header
  net_prependNetHeaderForForward(pkt, net_header);

  queue_freePacketBuffer(msg);
  net_forward(pkt, net_header);
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
