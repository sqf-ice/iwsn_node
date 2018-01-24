/**
\brief General IWSN 03-Network 'monitor' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "monitor.h"
#include "idmanager.h"
#include "neighbors.h"
#include "packet.h"
#include "ieee802154.h"
#include "ieee802154e.h"
#include "res.h"
#include "scheduler.h"
#include "tsch.h"

#ifdef HAVE_SERIAL
#include "serial.h"
#endif

#ifdef HAVE_USB
#include "usbAPI.h"
#endif
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

monitor_vars_t monitor_vars;
//=========================== prototypes ======================================

void 	monitor_receiveFromSink(QueueEntry_t *msg);	//v3
void    monitor_timer_cb();
//=========================== main ============================================

//=========================== public ==========================================

void monitor_init()
{
  // initailize local variables
  memset(&monitor_vars, 0, sizeof(monitor_vars_t));

  monitor_vars.periodMaintenance = 1000;  // fires every 3 sec on average
//  monitor_vars.timerId = timers_start(monitor_vars.periodMaintenance,
//                                      TIMER_PERIODIC, TIME_MS,
//                                      monitor_timer_cb);
}

void monitor_reveive(QueueEntry_t *msg)
{
	uint8_t subType = msg->payload[1];
	switch(subType)
	{
		case SLOT_COLLISION_NOTIFY:
			monitor_receiveFromSink(msg);
			break;
		default:
			break;
	}
}

void monitor_sendSlotCollisionNotify(uint8_t* nodeAddr)
{
	uint8_t nodeNum = nodeAddr[0];
	uint8_t* slotTable = ieee802154e_getSlotTable();
	QueueEntry_t*   pkt[nodeNum];
	uint8_t         temp_8b,i;
	iwsn_addr_t     tempAddr;
	tempAddr.type = ADDR_16B;
	tempAddr.addr_16b[0] = 0;
	tempAddr.addr_16b[1] = ADDRESS_HIGH;
	for(i = 1; i <= nodeNum; i++)
	{
		if (queue_getAvailableLength() <= 0 ||
				(pkt[i-1] = queue_getFreePacketBuffer(COMPONENT_NOTIFICATION)) == NULL)
		{
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_RES, ERR_NO_FREE_PACKET_BUFFER,
                       (errorparameter_t)0,
                       (errorparameter_t)0);
#endif
      	  return;
		}
		tempAddr.type = ADDR_16B;
		tempAddr.addr_16b[0] = 0;
		tempAddr.addr_16b[1] = ADDRESS_HIGH;
		tempAddr.addr_16b[1] |=  nodeAddr[i];
	    // declare ownership over that packet
	    pkt[i-1]->creator = COMPONENT_NOTIFICATION;
	    pkt[i-1]->owner   = COMPONENT_NOTIFICATION;

	    // reserve space for MON-specific header
	    packet_reserveHeaderSize(pkt[i-1], 10);

	    pkt[i-1]->payload[0] = NET_TYPE_NODE_NOTIFY;
	    pkt[i-1]->payload[1] = SLOT_COLLISION_NOTIFY;
	    memcpy(&pkt[i-1]->payload[2], &tempAddr, sizeof(iwsn_addr_t));
	    memcpy(&pkt[i-1]->payload[5], slotTable, SLOT_TABLE_LENGTH);

	    pkt[i-1]->l3_pduSpecifier = NET_TYPE_NODE_NOTIFY;
	    memcpy(&pkt[i-1]->l3_destinationORsource, &tempAddr, sizeof(iwsn_addr_t));

	    // some l2 information about this packet
	    temp_8b  = 0;
	    temp_8b |= (MAC_PDU_PKT_TYPE_DATA << MAC_PDU_PKT_TYPE)  & 0x07;
	    temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;

	    pkt[i-1]->l2_frameType                     = IEEE154_TYPE_DATA;
	    pkt[i-1]->l2_pduSpecifier                  = temp_8b;
	    pkt[i-1]->l2_shared                        = true;

	    if(net_send(pkt[i-1], NET_SEND_LOWER_TYPE, NULL) == E_FAIL)
	    {
	    	queue_freePacketBuffer(pkt[i-1]);
	    }
	    //update the slotTable or the collision notification always send
	    ieee802154e_updateNodeSlotInTable(SLOT_DEL, &tempAddr, 0);
	}
}

void monitor_checkChild()
{
	uint8_t myRole = idmanager_getMyRole();
	if(ieee802154e_isWork() == true)
	{
		if(idmanager_isGateway(myRole))
		{
			neighbors_sinkCheckChild();
		}else{
			neighbors_nodeCheckChildTimeout();
		}
	}
}
//=========================== private =========================================

void monitor_receiveFromSink(QueueEntry_t *msg)	//v3
{
	uint8_t* slotTable = ieee802154e_getSlotTable();
	tsch_updateMydedicatedSlotoffset(0xFF);
	memcpy(slotTable, &msg->payload[5], SLOT_TABLE_LENGTH);
	queue_freePacketBuffer(msg);
}

void monitor_timer_cb()
{
	if (ieee802154e_isWork() == true)
	{
		//scheduler_push_task(monitor_checkChild, TASKPRIO_NETWORK);
	}
}

//=========================== interrupt handlers ==============================
