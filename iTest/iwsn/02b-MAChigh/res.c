/**
\brief General IWSN 02bMAClow-high 'res' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================
//#include "wdt.h"
#include "res.h"
#include "packet.h"
#include "idmanager.h"
#include "scheduler.h"
#include "neighbors.h"
#include "network.h"
#include "cmd.h"
#include "monitor.h"	//v2
#include "board.h"

#ifdef HAVE_SERIAL
#include "serial.h"
#endif

#ifdef HAVE_USB
#include "usbAPI.h"
#endif
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

res_vars_t res_vars;
adv_add_info_t adv_add_info;

//=========================== prototypes ======================================

error_t res_send_internal(QueueEntry_t* msg);
void    sendAdv();
void    sendSync();
void    res_timer_cb();
void    timers_res_fired();
uint8_t res_getPktType(uint8_t specifier);
void    res_receiveFromSerial(uint8_t *buffer, uint8_t length);
void 	res_receiveFromUSBEthernet();
void 	res_receiveFromUsb();

void res_usbCon_cb();
void res_usbConnect_send();
void res_usbConnect_receive();
//=========================== main ============================================

//=========================== public ==========================================

void res_init()
{
  uint8_t myRole = idmanager_getMyRole();
  
  res_vars.periodMaintenance = 500;  // fires every 1 sec on average
  res_vars.dsn               = 0;
  res_vars.limit_adv         = 0;
  res_vars.limit_sync        = 0;
  res_vars.usb_con_flag		 = true;
  res_vars.usb_checkSeq      = 0;
  res_vars.checkLeave		 = 0;
  res_vars.checkResetSink    = 0;
  if (idmanager_isSyncSrc(myRole) == true || 
      idmanager_isProxy(myRole) == true)
  {
    res_vars.timerId = timers_start(res_vars.periodMaintenance,
                                    TIMER_PERIODIC, TIME_MS,
                                    res_timer_cb);
#ifdef HAVE_USB
    res_vars.usb_con_timerId = timers_start(2000,
    								TIMER_PERIODIC,TIME_MS,
    								res_usbCon_cb);
#endif
  }

  // Initilize adv additional information
  res_vars.advAddInfo = 0;
  res_vars.counter1 = 0;
  res_vars.counter2 = 0;
  res_vars.can_cnt = 0;
  res_vars.enable_wdt = false;
  memset(&adv_add_info, 0, sizeof(adv_add_info_t));
#ifdef HAVE_USB
#if SET_USB_FUNC == USB_ETHERNET
  usbeth_setRxDataCallbacks(res_receiveFromUsb);
#endif
#endif
}

void res_usbCon_cb()
{
	if (ieee802154e_isWork() == true)
	{
		if(++res_vars.checkResetSink > 1800)
		{
			board_reset();
		}
		scheduler_push_task(res_usbConnect_send,TASKPRIO_RES);
	}
}


void res_usbConnect_send()
{
	  uint8_t temp_8b;
	  QueueEntry_t* pkt;
	  iwsn_addr_t*  myAddr    = idmanager_getMyID(ADDR_16B);
	  iwsn_addr_t*  parent    = neighbors_getMyParentAddress();

	  if(res_vars.usb_con_flag == false)
	  {
#ifdef HAVE_USB
		  res_vars.usb_con_flag = true;
		  timers_stop(res_vars.usb_con_timerId);
		  USBHwReset();
		  USBInit();
		  timers_restart(res_vars.usb_con_timerId);
		  //HandleUsbRNDISClassReset(0);
	      return;
#endif
	  }

	  if (queue_getAvailableLength() <= 0 ||
	      (pkt = queue_getFreePacketBuffer(COMPONENT_NETWORK)) == NULL)
	  {
	#ifdef HAVE_DEBUG
	    debug_printError(COMPONENT_NETWORK, ERR_NO_FREE_PACKET_BUFFER,
	                     (errorparameter_t)0,
	                     (errorparameter_t)0);
	#endif
	  }

	  // decare ownership over that packet
	  pkt->creator = COMPONENT_RES;
	  pkt->owner   = COMPONENT_RES;

	  // reserve space for JOIN_REQ-specific content
	  packet_reserveHeaderSize(pkt, 7);

	  pkt->payload[0] = NET_TYPE_USB_CONNECT;
	  pkt->payload[1] = USB_CONNECT_SINK_TO_MANAGER;
	  pkt->payload[2] = 0;   // src count
	  memcpy(&(pkt->payload[3]), &(myAddr->addr_16b[0]), 2);
	  pkt->payload[5] = (res_vars.usb_checkSeq >> 8) & 0xFF;
	  pkt->payload[6] = res_vars.usb_checkSeq & 0xFF;


	  if (parent != NULL)
	  {
	    memcpy(&pkt->l3_destinationORsource, parent, sizeof(iwsn_addr_t));
	  }

	  // some l2 information about this packet
	  temp_8b  = 0;
	  temp_8b |= (MAC_PDU_PKT_TYPE_DATA << MAC_PDU_PKT_TYPE)  & 0x07;
	  temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;

	  pkt->l2_frameType                     = IEEE154_TYPE_DATA;
	  pkt->l2_pduSpecifier                  = temp_8b;
	  pkt->l2_shared                        = true;


	  packet_reserveHeaderSize(pkt, 2);
	  memcpy(&(pkt->payload[0]), &myAddr->addr_16b[0], 2);

#ifdef HAVE_USB
#if SET_USB_FUNC == USB_ETHERNET
	  usbeth_send(&(pkt->payload[0]), pkt->length);
#endif
#endif
	  queue_freePacketBuffer(pkt);

	  res_vars.usb_con_flag = false;
}

void res_usbConnect_receive()
{
	res_vars.usb_con_flag = true;
	res_vars.usb_checkSeq++;
	if(res_vars.usb_checkSeq == 0xFFFF)
	{
		res_vars.usb_checkSeq = 0;
	}
}



//======= from upper layer

void res_receiveFromUsb()
{
	scheduler_push_task(res_receiveFromUSBEthernet,TASKPRIO_RES);
}

error_t res_send(QueueEntry_t *msg)
{
  msg->owner        = COMPONENT_RES;
  msg->l2_frameType = IEEE154_TYPE_DATA;
  return res_send_internal(msg);
}

//======= from lower layer

void task_resNotifNewFrame()
{
  if (ieee802154e_isSynch() == true)
  {
    sendAdv();
    if(res_vars.enable_wdt == false)
    {
	    //wdt_start();
	    res_vars.enable_wdt = true;
    }
  }
}

void task_resNotifSendDone()
{
  QueueEntry_t* msg;
  asn_t* curASN = ieee802154e_getCurASN();
  // get recently-sent packet from queue
  msg = queue_resGetSentPacket();
  if (msg == NULL) 
  {
    // abort
    return;
  }
  // declare it as mine
  msg->owner = COMPONENT_RES;
  // indicate transmission (to update statistics)

  // send the packet to where it belongs
  if (msg->creator == COMPONENT_RES)
  {
    if ((msg->l2_pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ADV)
    {

      //wdt_feed();

      if (res_vars.limit_adv > 0)
      {
        res_vars.limit_adv--;
      }
    }
    else if ((msg->l2_pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_SYNC)
    {
      if (res_vars.limit_sync > 0)
      {
        res_vars.limit_sync--;
      }
    }
    
    // discard packets this component has created
    queue_freePacketBuffer(msg);
  }
  else
  {
	if (msg->l2_sendDoneError == E_SUCCESS)
	{
		neighbors_indicateTx(&(msg->l2_nextORpreviousHop), curASN);
	}
    // send the rest up the stack
    net_sendDone(msg, msg->l2_sendDoneError);
  }
}

void task_resNotifReceive()
{
  QueueEntry_t* msg;
  
  // get received packet from message queue
  msg = queue_resGetReceivedPacket();
  if (msg == NULL)
  {
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_RES, ERR_NO_RECEIVED_PACKET,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    // abort
    return;
  }
  
  // declare it as mine
  msg->owner = COMPONENT_RES;
  
  // indicate reception (to update statistics)

  
  // send the packet up the stack, if it qualifies
  switch (msg->l2_frameType)
  {
    case IEEE154_TYPE_BEACON:
      // free the packet's RAM memory
      queue_freePacketBuffer(msg);
      break;
    case IEEE154_TYPE_DATA:
      if ((msg->length > 0) &&
          (res_getPktType(msg->l2_pduSpecifier) == MAC_PDU_PKT_TYPE_DATA))
      {
    	// update the asn of neighbor
    	neighbors_indicateRx(&(msg->l2_nextORpreviousHop),
							 msg->l1_rssi,
							 &msg->l2_asn);
        // send to upper layer
        net_receive(msg);
      }
      else if ((msg->length > 0) &&
               (res_getPktType(msg->l2_pduSpecifier) == MAC_PDU_PKT_TYPE_ADV) &&
               (neighbors_isPreferredParent(&msg->l2_nextORpreviousHop) == true))
      {
        if (ieee802154e_isWork() == true) //改成已分配好dedicated时隙了
        {
            // need to receive additional information
            res_addInfoStoreFromAdv(msg);
        }
        else
        {
          // free the packet's RAM memory
          queue_freePacketBuffer(msg);
        }
      }
      else
      {
        // free up the RAM
        queue_freePacketBuffer(msg);
      }
      
      break;
    case IEEE154_TYPE_CMD:
      // free the packet's RAM memory
      queue_freePacketBuffer(msg);
      break;
    case IEEE154_TYPE_ACK:
      // free the packet's RAM memory
      queue_freePacketBuffer(msg);
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_RES, ERR_MSG_UNKNOWN_TYPE,
                       (errorparameter_t)msg->l2_frameType,
                       (errorparameter_t)0);
#endif
      // free the packet's RAM memory
      queue_freePacketBuffer(msg);
      break;
  }
}

uint8_t res_getCurDsn()
{
  return res_vars.dsn++;
}

bool res_wdtEnable()
{
	return res_vars.enable_wdt;
}

void res_addInfoStoreFromMsg(QueueEntry_t* msg)
{
  adv_add_info_t*   ptr;
  ptr = (adv_add_info_t *)&(msg->payload[5]);
  adv_add_info.type   = ptr->type;
  adv_add_info.length = ptr->length;
  memcpy(&adv_add_info.info[0], &ptr->info[0], ptr->length);
  res_vars.advAddInfo =1;

  queue_freePacketBuffer(msg);
}

void res_addInfoStoreFromAdv(QueueEntry_t* advFrame)
{
  adv_add_info_t* ptr = (adv_add_info_t *)advFrame->payload;

  adv_add_info.type   = ptr->type;
  adv_add_info.length = ptr->length;
  memcpy(&adv_add_info.info[0], &ptr->info[0], ptr->length);
  res_vars.advAddInfo =1;

  // Execute the cmd
  cmd_execute(ptr->type, ptr->length,&ptr->info[0]);

  queue_freePacketBuffer(advFrame);
}

void res_addInfoWriteToAdv(QueueEntry_t* advFrame)
{
  adv_add_info_t* ptr = (adv_add_info_t *)&advFrame->l2_payload[ADV_PAYLOAD_LENGTH];

  if (res_vars.advAddInfo == 0)
  {
    return;
  }
  else
  {
    ptr->type   = adv_add_info.type;
    ptr->length = adv_add_info.length;
    memcpy(&ptr->info[0], &adv_add_info.info[0], adv_add_info.length);
    res_vars.advAddInfo = 0;
  }
}

error_t adv_add_send(QueueEntry_t *msg)
{
  // change owner to IEEE802154E fetches it from queue
  msg->owner        = COMPONENT_ADV;
  msg->l2_frameType = IEEE154_TYPE_DATA;

  // assign a number of retries
  if (packet_isBroadcastMulticast(&(msg->l2_nextORpreviousHop)) == true)
  {
    msg->l2_retriesLeft = 1;
  }
  // record this packet's dsn (for matching the ACK)
  msg->l2_dsn = res_getCurDsn();
  // this is a new packet which I never attempted to send
  msg->l2_numTxAttempts = 0;
  // transmit with the default TX power
  msg->l1_txPower = TX_POWER;
  // record the location, in the packet, where the l2 payload starts
  msg->l2_payload = msg->payload;
  // add a IEEE802.15.4 header
  ieee802154_prependHeader(msg,
                           msg->l2_frameType,
                           IEEE154_SEC_NO_SECURITY,
                           msg->l2_dsn,
                           &(msg->l2_nextORpreviousHop));

  return E_SUCCESS;
}

void adv_add_send_internal(QueueEntry_t* msg)
{
  uint8_t       length = adv_getUnusedLength(msg);

  // toss the unnecessary bytes
  packet_tossFooter(msg, length);

  // reserve space for 2-byte CRC
  packet_reserveFooterSize(msg, 2);

  // change owner to IEEE802154E fetches it from queue
  msg->owner        = COMPONENT_ADV_TO_IEEE802154E;
}

uint8_t adv_getUnusedLength(QueueEntry_t* msg)
{
  adv_add_info_t* ptr = (adv_add_info_t*)&msg->l2_payload[ADV_PAYLOAD_LENGTH];

  if (ptr->type == ADV_ADD_INFO_TYPE_NULL)
  {
    return sizeof(adv_add_info_t);
  }
  else
  {
    return (MAX_LENGTH_ADV_ADD_INFO - ptr->length);
  }
}

void res_resetCheckLeave()
{
	res_vars.checkLeave = 0;
}

void res_accumulateLeave()
{
	if(++res_vars.checkLeave == MAX_LOSS_LEAVE_COUNTS)
	{
#if SET_NODE_TYPE
#else
	board_reset();
#endif
	}
}


//=========================== private =========================================

/**
\brief Transfer packet to MAC.

This function adds a IEEE802.15.4 header to the packet and leaves it the 
Queue buffer. The very last thing it does is assigning this packet to the 
virtual component COMPONENT_RES_TO_IEEE802154E. Whenever it gets a change,
IEEE802154E will handle the packet.

\param [in] msg The packet to the transmitted

\returns E_SUCCESS iff successful.
*/

error_t res_send_internal(QueueEntry_t* msg)
{
  // assign a number of retries
  if (packet_isBroadcastMulticast(&(msg->l2_nextORpreviousHop)) == true)
  {
    msg->l2_retriesLeft = 1;
  }
  else
  {
    msg->l2_retriesLeft = TXRETRIES;
  }
  // record this packet's dsn (for matching the ACK)
  msg->l2_dsn = res_vars.dsn++;
  // this is a new packet which I never attempted to send
  msg->l2_numTxAttempts = 0;
  // transmit with the default TX power
  msg->l1_txPower = TX_POWER;
  // record the location, in the packet, where the l2 payload starts
  msg->l2_payload = msg->payload;
  // add a IEEE802.15.4 header
  ieee802154_prependHeader(msg,
                           msg->l2_frameType,
                           IEEE154_SEC_NO_SECURITY,
                           msg->l2_dsn,
                           &(msg->l2_nextORpreviousHop));
  // reserve space for 2-byte CRC
  packet_reserveFooterSize(msg, 2);
  // change owner to IEEE802154E fetches it from queue
  msg->owner  = COMPONENT_RES_TO_IEEE802154E;
  return E_SUCCESS;
}

/**
\brief Send an advertisement.

This is one of the MAC managament tasks. This function inlines in the
timers_res_fired() function, but is declared as a separate function for better
readability of the code.
*/
port_INLINE void sendAdv()
{
  uint8_t temp_8b;
  QueueEntry_t* advPkt;
  
  if (res_vars.limit_adv >= 2)
  {
    return;
  }
  
  if (queue_getAvailableLength() > 0)
  {
    // get a free packet buffer
    advPkt = queue_getFreePacketBuffer(COMPONENT_RES);
    if (advPkt == NULL)
    {
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_RES, ERR_NO_FREE_PACKET_BUFFER,
                       (errorparameter_t)0,
                       (errorparameter_t)0);
#endif
      return;
    }
    
    // declare ownership over that packet
    advPkt->creator = COMPONENT_RES;
    advPkt->owner   = COMPONENT_RES;

    // reserve space for ADV-specific header
    packet_reserveHeaderSize(advPkt, ADV_PAYLOAD_LENGTH + sizeof(adv_add_info_t));
    // the actual value of the current ASN and join Info will be written by the
    // IEEE802.15.4e when transmitting
    
    // some l2 information about this packet
    temp_8b  = 0;
    temp_8b |= (MAC_PDU_PKT_TYPE_ADV << MAC_PDU_PKT_TYPE)     & 0x07;
    temp_8b |= (MAC_PDU_PRIORITY_NORMAL << MAC_PDU_PRIORITY)  & 0x30;
    
    advPkt->l2_frameType                     = IEEE154_TYPE_DATA;
    advPkt->l2_pduSpecifier                  = temp_8b;
    advPkt->l2_nextORpreviousHop.type        = ADDR_16B;
    advPkt->l2_nextORpreviousHop.addr_16b[0] = 0xff;
    advPkt->l2_nextORpreviousHop.addr_16b[1] = 0xff;
    
    // put in queue for MAC to handle
//    res_send_internal(advPkt);

    advPkt->l2_payload = advPkt->payload;
    advPkt->l2_payload[ADV_PAYLOAD_LENGTH] = ADV_ADD_INFO_TYPE_NULL;
    if (res_vars.advAddInfo == 1)
    {
      res_addInfoWriteToAdv(advPkt);
      adv_add_send(advPkt);
      adv_add_send_internal(advPkt);
    }
    else
    {
      adv_add_send(advPkt);
      adv_add_send_internal(advPkt);
    }

    res_vars.limit_adv++;
  }
}

/**
\brief Send an synchronization.

This is one of the MAC managament tasks. This function inlines in the
timers_res_fired() function, but is declared as a separate function for better
readability of the code.
*/
port_INLINE void sendSync()
{
  uint8_t temp_8b;
  QueueEntry_t* syncPkt;

  if (res_vars.limit_sync >= 2)
  {
    return;
  }

  if (queue_getAvailableLength() <= 0 ||
      (syncPkt = queue_getFreePacketBuffer(COMPONENT_RES)) == NULL)
  {

#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_RES, ERR_NO_FREE_PACKET_BUFFER,
                       (errorparameter_t)0,
                       (errorparameter_t)0);
#endif
      return;
  }

  // declare ownership over that packet
  syncPkt->creator = COMPONENT_RES;
  syncPkt->owner   = COMPONENT_RES;

  // reserve space for SYNC-specific header
  packet_reserveHeaderSize(syncPkt, SYNC_PAYLOAD_LENGTH);
  // the actual value of the current ASN will be written by the
  // IEEE802.15.4e when transmitting

  // some l2 information about this packet
  temp_8b  = 0;
  temp_8b |= (MAC_PDU_PKT_TYPE_SYNC << MAC_PDU_PKT_TYPE)    & 0x07;
  temp_8b |= (MAC_PDU_PRIORITY_NORMAL << MAC_PDU_PRIORITY)  & 0x30;

  syncPkt->l2_frameType                     = IEEE154_TYPE_DATA;
  syncPkt->l2_pduSpecifier                  = temp_8b;
  syncPkt->l2_nextORpreviousHop.type        = ADDR_16B;
  syncPkt->l2_nextORpreviousHop.addr_16b[0] = 0xff;
  syncPkt->l2_nextORpreviousHop.addr_16b[1] = 0xff;

  // put in queue for MAC to handle
  res_send_internal(syncPkt);
  res_vars.limit_sync++;
}

void res_timer_cb()
{
  scheduler_push_task(timers_res_fired, TASKPRIO_RES);
}

/**
\brief Timer handlers which triggers MAC management task.

This function is called in task context by the scheduler after the RES timer
has fired. This timer is set to fire every second, on average.

The body of this function executes the MAC management task.
*/
void timers_res_fired()
{
  uint8_t       myRole = idmanager_getMyRole();
  QueueEntry_t* pkt;
  iwsn_addr_t*  neighbor;
  
  if ((ieee802154e_isWork() == true) &&
      (idmanager_isSyncSrc(myRole) == true || 
       idmanager_isProxy(myRole) == true))
  {
//    sendAdv();
#ifdef HAVE_MUlTI_GATEWAY
    sendSync();
#endif
  }
  
  if ((pkt = queue_getTimeoutPacket()) != NULL)
  {
    queue_freePacketBuffer(pkt);
  }
  if ((neighbor = neighbors_getTimeoutPeerNeighbor(ADDR_16B)) != NULL)
  {
    neighbors_removeNeighbor(neighbor);
  }
}

uint8_t res_getPktType(uint8_t specifier)
{
  return (specifier >> MAC_PDU_PKT_TYPE) & 0x07;
}

port_INLINE void res_receiveFromSerial(uint8_t *buffer, uint8_t length)
{
  QueueEntry_t* pkt;
  uint8_t       temp_8b;

  if (buffer == NULL || length <= 0)
  {
    return;
  }

  if (queue_getAvailableLength() <= 2 ||
      (pkt = queue_getFreePacketBuffer(COMPONENT_RES)) == NULL)
  {
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_RES, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    return;
  }

  // declare ownership over that packet
  pkt->creator  = COMPONENT_RES;
  pkt->owner    = COMPONENT_RES;

  packet_reserveHeaderSize(pkt, length);
  memcpy(&(pkt->payload[0]), &(buffer[0]), length);

  temp_8b  = 0;
  temp_8b |= (MAC_PDU_PKT_TYPE_DATA << MAC_PDU_PKT_TYPE)  & 0x07;
  temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;

  pkt->l2_frameType                     = IEEE154_TYPE_DATA;
  pkt->l2_pduSpecifier                  = temp_8b;
  pkt->l2_shared                        = false;

  if (net_prependNetHeader(pkt) == E_SUCCESS)
  {
    net_receive(pkt);
  }
  else
  {
    queue_freePacketBuffer(pkt);
  }
}

#ifdef HAVE_USB
port_INLINE void res_receiveFromUSBEthernet()
{
  QueueEntry_t* pkt;
  uint8_t       temp_8b;
  usbeth_rx_msg_t* usbeth_msg = usbeth_getRxMsg();

  if (usbeth_msg == NULL)
  {
    return;
  }

  if(usbeth_msg->pkt[0] == NET_TYPE_USB_CONNECT &&
		  usbeth_msg->pkt[1] == 0 )
  {
	  res_usbConnect_receive();

	  usbeth_FreeRxCache(usbeth_msg);
	  return;
  }

  if (queue_getAvailableLength() <= 2 ||
      (pkt = queue_getFreePacketBuffer(COMPONENT_RES)) == NULL)
  {
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_RES, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    return;
  }

  // declare ownership over that packet
  pkt->creator  = COMPONENT_RES;
  pkt->owner    = COMPONENT_RES;

  packet_reserveHeaderSize(pkt, usbeth_msg->length);
  memcpy(&(pkt->payload[0]), &(usbeth_msg->pkt[0]), usbeth_msg->length);

  usbeth_FreeRxCache(usbeth_msg);


  temp_8b  = 0;
  temp_8b |= (MAC_PDU_PKT_TYPE_DATA << MAC_PDU_PKT_TYPE)  & 0x07;
  temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;

  pkt->l2_frameType                     = IEEE154_TYPE_DATA;
  pkt->l2_pduSpecifier                  = temp_8b;
  pkt->l2_shared                        = false;

  if (net_prependNetHeader(pkt) == E_SUCCESS)
  {
    net_receive(pkt);
  }
  else
  {
    queue_freePacketBuffer(pkt);
  }
}
#endif


//=========================== interrupt handlers ==============================

