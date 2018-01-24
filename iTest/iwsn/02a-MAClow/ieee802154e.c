/**
\brief General IWSN 02aMAClow-layer 'ieee802154e' definitions

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================
#ifdef HAVE_CONFIG
#include "config.h"
#endif

#include <string.h>
//#include "wdt.h"
#include "ieee802154.h"
#include "ieee802154e.h"
#include "idmanager.h"
#include "rtimer.h"
#include "radio.h"
#include "board.h"
#include "packet.h"
#include "neighbors.h"
#include "scheduler.h"
#include "res.h"
//#include "can.h"
//#include "can_trx.h"
#include "monitor.h"	//rev.2 monitor
//#include "uart_trx.h"

#ifdef HAVE_SERIAL
#include "serial.h"
#endif

#ifdef HAVE_USB
#include "usbAPI.h"
#endif

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

ieee802154e_vars_t ieee802154e_vars;
ieee802154e_advs_t ieee802154e_advs;

uint8_t uart_sendcommand_frequency=0;

#ifdef DEBUG_TIRI
ti_t ti;
ri_t ri;
#endif
//=========================== prototypes ======================================

// SYNCHRONIZING      ,
void     activity_synchronize_newSlot();
void     activity_synchronize_startOfFrame(PORT_TIMER_WIDTH capturedTime);
void     activity_synchronize_endOfFrame(PORT_TIMER_WIDTH capturedTime);
// TRX
void     activity_ti1ORri1();
// TX
void     activity_ti2();
void     activity_tie1();
void     activity_ti3();
void     activity_tie2();
void     activity_ti4(PORT_TIMER_WIDTH capturedTime);
void     activity_tie3();
void     activity_ti5(PORT_TIMER_WIDTH capturedTime);
void     activity_ti6();
void     activity_tie4();
void     activity_ti7();
void     activity_tie5();
void     activity_ti8(PORT_TIMER_WIDTH capturedTime);
void     activity_tie6();
void     activity_ti9(PORT_TIMER_WIDTH capturedTime);
// RX
void     activity_ri2();
void     activity_rie1();
void     activity_ri3();
void     activity_rie2();
void     activity_ri4(PORT_TIMER_WIDTH capturedTime);
void     activity_rie3();
void     activity_ri5(PORT_TIMER_WIDTH capturedTime);
void     activity_ri6();
void     activity_rie4();
void     activity_ri7();
void     activity_rie5();
void     activity_ri8(PORT_TIMER_WIDTH capturedTime);
void     activity_rie6();
void     activity_ri9(PORT_TIMER_WIDTH capturedTime);
void     endSlot();
// frame validity check
bool     isValidAdv(ieee802154_header_iht*     ieee802154_header);
bool     isValidSync(ieee802154_header_iht*    ieee802154_header);
bool     isValidRxFrame(ieee802154_header_iht* ieee802154_header);
bool     isValidAck(ieee802154_header_iht*     ieee802154_header,
                    QueueEntry_t*              packetSent);

// ASN handling
void     incrementAsnOffset();
//void     asnWriteToAdvInfo();
void     asnStoreFromAdv(QueueEntry_t* advFrame);
bool     asnCheckFromAdv(QueueEntry_t* advFrame);   //v3
uint16_t ieee802154e_getCurFrame(asn_t asn);
void     AdvInfoWriteToAdv(QueueEntry_t* advFrame);

// synchronization
void     synchronizePacket(PORT_TIMER_WIDTH timeReceived);
void     synchronizeAck(PORT_SIGNED_INT_WIDTH timeCorrection);
// notifying upper layer
void     notif_sendDone(QueueEntry_t* packetSent, error_t error);
void     notif_receive(QueueEntry_t* packetReceived);
uint8_t  calculateChannel(uint8_t channelOffset);
void     changeState(ieee802154e_state_t newstate);

//rev.3
void ieee802154e_advInit();
void ieee802154e_updateDSlotTable(uint8_t* DSlotTable); //v3
void ieee802154e_updateMyparentDSlotTable(uint8_t* DSlotTable); //v3
void ieee802154e_updateNoUseSlotInTable(uint8_t* DSlotTable);  //v3
void ieee802154e_selectDSlotoffset(); //v3
int8_t slotTableShellSort(nodeSlotTable_t* table, uint8_t length); //v3
int8_t expandDSlotTable(uint8_t* Dtable, uint8_t length, nodeSlotTable_t* result);   //v3
void checkDSlotContentionNode();

//=========================== main ============================================

//=========================== public ==========================================

void ieee802154e_init()
{
  // Initialize variables
  memset(&ieee802154e_vars, 0, sizeof(ieee802154e_vars_t));
  ieee802154e_advInit();
  // Initialize synchronization status

  ieee802154e_changeIsSync(false);
  // Set callback functions for the radiotimer
  rtimer_setOverflowCb(isr_ieee802154e_newSlot);
  rtimer_setCompareCb(isr_ieee802154e_timer);
  
  // Set callback functions for the radio
  radio_setStartFrameCb(isr_ieee802154e_startOfFrame);
  radio_setEndFrameCb(isr_ieee802154e_endOfFrame);
}


bool ieee802154e_isSynch()
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  if (ieee802154e_vars.isSync == true)
  {
    ENABLE_INTERRUPTS();
    return true;
  }
  ENABLE_INTERRUPTS();
  return false;
}

bool ieee802154e_isWork()
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  if (ieee802154e_vars.isWork == true)
  {
    ENABLE_INTERRUPTS();
    return true;
  }
  ENABLE_INTERRUPTS();
  return false;
}

bool ieee802154e_isNeighborScand()
{
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	if(ieee802154e_vars.listenForAdvs > 101)
	{
		ENABLE_INTERRUPTS();
		return true;
	}
	ENABLE_INTERRUPTS();
	return false;
}

asn_t*  ieee802154e_getCurASN()
{
  return &ieee802154e_vars.asn;
}

uint16_t ieee802154e_getCurFrame(asn_t asn)
{
	return ((asn.byte4 * 65536 * 65536ULL +
	          asn.bytes2and3 * 65536 +
	          asn.bytes0and1) / tsch_getFrameLength());
}

uint8_t* ieee802154e_getSlotTable()
{
	return ieee802154e_advs.dSlotTable;
}


void ieee802154e_listenForAdvs()
{ 
  ieee802154e_vars.listenForAdvs++;
}


void ieee802154e_updateAdvInfo()   //v3
{
	iwsn_addr_t* tempAddr;
	ieee802154e_advs.asn[0]        = (ieee802154e_vars.asn.bytes0and1       & 0xFF);
	ieee802154e_advs.asn[1]        = (ieee802154e_vars.asn.bytes0and1 / 256 & 0xFF);
	ieee802154e_advs.asn[2]        = (ieee802154e_vars.asn.bytes2and3       & 0xFF);
	ieee802154e_advs.asn[3]        = (ieee802154e_vars.asn.bytes2and3 / 256 & 0xFF);
	ieee802154e_advs.asn[4]        =  ieee802154e_vars.asn.byte4;
	tempAddr = neighbors_getSinkAddress();
	memcpy(&ieee802154e_advs.sinkAddr, tempAddr, sizeof(iwsn_addr_t));
	tempAddr = neighbors_getMyParentAddress();
	memcpy(&ieee802154e_advs.parentAddr, tempAddr, sizeof(iwsn_addr_t));
	tempAddr = neighbors_getMyChildAddress();
	memcpy(&ieee802154e_advs.childAddr, tempAddr, sizeof(iwsn_addr_t));
	ieee802154e_advs.mySlotoffset = tsch_getMyDedicatedSlotOffset();
	ieee802154e_advs.childSlotoffset = tsch_getMyChildSlotOffset();
}

//update adv's address information (parent address, child address)
void ieee802154e_updateAdvAddrInfo(uint8_t addrType, iwsn_addr_t* addr)  //v3
{
	switch(addrType)
	{
	case PARENT_ADDR:
		memcpy(&ieee802154e_advs.parentAddr, addr, sizeof(iwsn_addr_t));
		break;
	case CHILD_ADDR:
		memcpy(&ieee802154e_advs.childAddr, addr, sizeof(iwsn_addr_t));
		break;
	default:
		break;
	}
}

void ieee802154e_updateNodeSlotInTable(uint8_t type, iwsn_addr_t* address, slotOffset_t slot)  //v3
{
	if(address == NULL)
	{
		return;
	}
	if(type == SLOT_ADD && ((slot < 0) || (slot > 9)))
	{
		return;
	}
	uint8_t addrMark = (address->addr_16b[1] & 0x0F) - 1;
	uint8_t mask1 = 0xF0, mask2 = 0x0F;
	if(!(addrMark % 2))
	{
		slot = slot << 4;
		mask1 = 0x0F;
	    mask2 = 0xF0;
	}
	switch(type)
	{
	case SLOT_ADD:
		ieee802154e_advs.dSlotTable[addrMark/2] &= mask1;
		ieee802154e_advs.dSlotTable[addrMark/2] += (slot & mask2);
		break;
	case SLOT_DEL:
		ieee802154e_advs.dSlotTable[addrMark/2] &= mask1;
		ieee802154e_advs.dSlotTable[addrMark/2] += mask2;
		break;
	default:
		break;
	}
}
//=========================== private =========================================
//initial the broadcast information
void ieee802154e_advInit()  //v3
{
	uint8_t myRole = idmanager_getMyRole();
	memset(&ieee802154e_advs, 0, sizeof(ieee802154e_advs_t));
	memset(&ieee802154e_advs.dSlotTable[0], 0xFF, sizeof(ieee802154e_advs.dSlotTable));
	ieee802154e_advs.role = myRole;
	ieee802154e_advs.channel = SYNCHRONIZING_CHANNEL;
}
// ====== SYNCHRONIZING
port_INLINE void activity_synchronize_newSlot()
{
  uint8_t myRole = idmanager_getMyRole();
  // I'm in the middle of receiving a packet
  if (ieee802154e_vars.state == S_SYNCRX)
  {
    return;
  }
  
  if (ieee802154e_vars.deSyncTimeout == 0)
  {
    ieee802154e_vars.deSyncTimeout    = DESYNCTIMEOUT;
    ieee802154e_vars.numDeSync++;
    if (ieee802154e_vars.numDeSync >= DESYNCLIMIT)
    {
      board_reset();
      return;
    }
  }
  ieee802154e_vars.deSyncTimeout--;
  
  // If this is the first time I call this function while not synchronized,
  // switch on the radio in Rx mode
  if (ieee802154e_vars.state != S_SYNCLISTEN)
  {
    // Change state
    changeState(S_SYNCLISTEN);
    
    // Turn off the radio (in case it wasn't yet)
    radio_rfOff();
    
    // Configure the radio to listen to the default synchronizing channel
    if (idmanager_isGateway(myRole) == true)
    {
      radio_setChannel(SYNCHRONIZING_CHANNEL);
      
      // Update record of current channel
      ieee802154e_vars.channel = SYNCHRONIZING_CHANNEL;
    }
    else
    {
      radio_setChannel(SYNCHRONIZING_CHANNEL);
      
      // Update record of current channel
      ieee802154e_vars.channel = SYNCHRONIZING_CHANNEL;
    }
    
    // Switch on the radio in Rx mode.
    radio_rxEnable();
    radio_rxNow();
  }
}

port_INLINE void activity_synchronize_startOfFrame(PORT_TIMER_WIDTH capturedTime)
{
  // Don't care about packet if I'm not listening
  if (ieee802154e_vars.state != S_SYNCLISTEN)
  {
    return;
  }
  
  // Change state
  changeState(S_SYNCRX);
  
  // stop the serial
#ifdef HAVE_SERIAL
  serial_stop();
#endif
  
  // record the captured time 
  ieee802154e_vars.lastCapturedTime = capturedTime;
   
  // record the captured time (for sync)
  ieee802154e_vars.syncCapturedTime = capturedTime;
}

port_INLINE void activity_synchronize_endOfFrame(PORT_TIMER_WIDTH capturedTime)
{
  ieee802154_header_iht ieee802154_header;
  ieee802154e_advs_t*   adv;
  uint8_t myRole = idmanager_getMyRole();
  // check state
  if (ieee802154e_vars.state != S_SYNCRX)
  {
#ifdef HAVE_DEBUG
    // log the error
    debug_printError(COMPONENT_IEEE802154E, ERR_WRONG_STATE_IN_ENDFRAME_SYNC,
                     (errorparameter_t)ieee154e_vars.state,
                     (errorparameter_t)0);
#endif
    // abort
    endSlot();
  }
  
  // change state
  changeState(S_SYNCPROC);
  
  // get a buffer to put the (received) frame in
  ieee802154e_vars.dataReceived = queue_getFreePacketBuffer(COMPONENT_IEEE802154E);
  if (ieee802154e_vars.dataReceived == NULL)
  {
#ifdef HAVE_DEBUG
    // log the error
    debug_printError(COMPONENT_IEEE802154E, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    // abort
    endSlot();
    return;
  }
  
  // declare ownership over that packet
  ieee802154e_vars.dataReceived->creator = COMPONENT_IEEE802154E;
  ieee802154e_vars.dataReceived->owner   = COMPONENT_IEEE802154E;
  
  // retrieve the received data frame from the radio's Rx buffer
  ieee802154e_vars.dataReceived->payload = &(ieee802154e_vars.dataReceived->packet[0]);
  radio_getReceivedFrame(       ieee802154e_vars.dataReceived->payload,
                               &ieee802154e_vars.dataReceived->length,
                         sizeof(ieee802154e_vars.dataReceived->packet),
                               &ieee802154e_vars.dataReceived->l1_rssi,
                               &ieee802154e_vars.dataReceived->l1_lqi,
                               &ieee802154e_vars.dataReceived->l1_crc);
  // toss CRC (2 last bytes)
  packet_tossFooter(ieee802154e_vars.dataReceived, LENGTH_CRC);
   
  /*
  The do-while loop that follows is a little parsing trick.
  Because it contains a while(0) condition, it gets executed only once.
  The behavior is:
  - if a break occurs inside the do{} body, the error code below the loop
    gets executed. This indicates something is wrong with the packet being 
    parsed.
  - if a return occurs inside the do{} body, the error code below the loop
    does not get executed. This indicates the received packet is correct.
  */ 
  do
  { // this "loop" is only executed once
    // break if invalid CRC
    if (ieee802154e_vars.dataReceived->l1_crc == 0)
    {
      // break from the do-while loop and execute abort code below
      break;
    }
    
    // parse the IEEE802.15.4 header
    ieee802154_retrieveHeader(ieee802154e_vars.dataReceived, &ieee802154_header);
    
    // store header details in packet buffer
    ieee802154e_vars.dataReceived->l2_frameType    = ieee802154_header.frameType;
    ieee802154e_vars.dataReceived->l2_dsn          = ieee802154_header.dsn;
    memcpy(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop), &(ieee802154_header.src), sizeof(iwsn_addr_t));
    ieee802154e_vars.dataReceived->l2_pduSpecifier = ieee802154_header.pduSpecifier;
    
    // toss the IEEE802.15.4 header
    packet_tossHeader(ieee802154e_vars.dataReceived, ieee802154_header.headerLength);
    
    // if I just received a valid ADV, handle
    if (isValidAdv(&ieee802154_header) == true)
    {
      // turn off the radio
      radio_rfOff();
      // synchronize (for the first time) to the sender's ADV
      synchronizePacket(ieee802154e_vars.syncCapturedTime);

      adv = (ieee802154e_advs_t *)&ieee802154e_vars.dataReceived->payload[0];
      // record the ASN from the ADV payload
      asnStoreFromAdv(ieee802154e_vars.dataReceived);
      if(idmanager_isGateway(myRole) == false)
      {
    	  // declare synchronized
          ieee802154e_changeIsSync(true);
          //update address of sink
          neighbors_updateMySinkAddr(&adv->sinkAddr);
          //update neighbor no matter whether it's my neighbor before
          neighbors_updateNeighborFromadv(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop),
        		  	  	  	  	  	  	  adv->role,
        		  	  	  	  	  	  	  &adv->parentAddr,
        		  	  	  	  	  	  	  &adv->childAddr,
        		  	  	  	  	  	  	  ieee802154e_vars.dataReceived->l1_rssi,
        		  	  	  	  	  	  	  adv->mySlotoffset);
      }else{
          //update neighbor no matter whether it's my neighbor before
    	  neighbors_sinkUpdateNeighbors(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop),
        		  	  	  	  	  	  	  adv->role,
        		  	  	  	  	  	  	  &adv->parentAddr,
        		  	  	  	  	  	  	  &adv->childAddr,
        		  	  	  	  	  	  	  ieee802154e_vars.dataReceived->l1_rssi,
        		  	  	  	  	  	  	  adv->mySlotoffset,
        		  	  	  	  	  	  	  adv->childSlotoffset);
    	  ieee802154e_updateNodeSlotInTable(SLOT_ADD, &(ieee802154e_vars.dataReceived->l2_nextORpreviousHop), adv->mySlotoffset);
    	  ieee802154e_updateNodeSlotInTable(SLOT_ADD, &adv->childAddr, adv->childSlotoffset);
      }


      // toss the ADV payload
      packet_tossHeader(ieee802154e_vars.dataReceived, ADV_PAYLOAD_LENGTH);
      
      // send received ADV up the stack so RES can update statistics (synchronizing)
      notif_receive(ieee802154e_vars.dataReceived);

      // clear local variable
      ieee802154e_vars.dataReceived = NULL;
      
      // official end of synchronization
      endSlot();
      
      // everything went well, return here not to execute the error code below
      return;
    }
  } while(0);
  
  // free the (invalid) received data buffer so RAM memory can be recycled
  queue_freePacketBuffer(ieee802154e_vars.dataReceived);
   
  // clear local variable
  ieee802154e_vars.dataReceived = NULL;
}

// ====== TRX
port_INLINE void activity_ti1ORri1()
{
  cellType_t  cellType;
  iwsn_addr_t neighbor;
  uint8_t myRole = idmanager_getMyRole();
#ifdef DEBUG_TIRI
  memset(&ti,0,sizeof(ti_t));
  memset(&ri,0,sizeof(ri_t));
  memset(&canti,0,sizeof(canti_t));
  memset(&canri,0,sizeof(canri_t));
#endif

  // increment ASN 
  incrementAsnOffset();

#ifdef HAVE_UART
#ifdef HAVE_UART_DEBUG
  if(ieee802154e_isWork() == true)
  {
	  if(getDevNum()==0){
		  uart_sendGetDevAddrCommand();
		  return;
	  }
	  if((ieee802154e_vars.slotOffset%(tsch_getFrameLength()/getDevNum()))==11)
//			  &&(uart_sendcommand_frequency++)%2==0)
		 // scheduler_push_task(uart_rxData_callback,TASKPRIO_RES);
		  scheduler_push_task(uart_sendCommand,TASKPRIO_RES);

  }
#endif
#endif
  if (ieee802154e_vars.slotOffset == 0)
  {
	  scheduler_push_task(monitor_checkChild, TASKPRIO_RES);
  }

  if (ieee802154e_vars.slotOffset == 11)
  {
    // post RES's a new frame task 
    scheduler_push_task(task_resNotifNewFrame, TASKPRIO_RES);
    if(ieee802154e_isWork() == false)
    {
    	ieee802154e_vars.scanSuperframe++;
    }
  }
  if(idmanager_isGateway(myRole) == false)
  {
	  if (ieee802154e_vars.deSyncTimeout == 0)
	  {
	    ieee802154e_vars.deSyncTimeout    = DESYNCTIMEOUT;
	    ieee802154e_changeIsSync(false);
	    ieee802154e_vars.numDeSync++;
	    if (ieee802154e_vars.numDeSync >= DESYNCLIMIT)
	    {
	      board_reset();
	      return;
	    }
	  }
	  ieee802154e_vars.deSyncTimeout--;
  }

  // if the previous slot took too long, we will not be in the right state
  if (ieee802154e_vars.state != S_SLEEP)
  {
#ifdef HAVE_DEBUG
    debug_printError(COMPONENT_IEEE802154E, ERR_WRONG_STATE_IN_STARTSLOT,
                     (errorparameter_t)ieee802154e_vars.state,
                     (errorparameter_t)ieee802154e_vars.slotOffset);
#endif
    // abort
    endSlot();
    return;
  }

  if (ieee802154e_vars.slotOffset == ieee802154e_vars.nextActiveSlotOffset)
  {
    // this is the next active slot
    
    // advance the schedule tsch
    tsch_advanceSlot();
    
    // find the next active slotOffset
    ieee802154e_vars.nextActiveSlotOffset    = tsch_getNextActiveSlotOffset();
  }
  else
  {
    // this is NOT the next active slot, abort the slot
    endSlot();
    return;
  }
  
  // check the schedule cell to see what type of slot this is
  cellType = tsch_getCurCellType();
  switch (cellType)
  {
    case CELLTYPE_ADV:
	  if(idmanager_isGateway(myRole) == false)
	  {
		  if(neighbors_isParentSelected() == false && ieee802154e_vars.scanSuperframe > 2)
		  {
			  neighbors_selectParent();  //V3
		  }
		  if(neighbors_isParentSelected() && tsch_isDSlotSelected() == false)
		  {
			  ieee802154e_selectDSlotoffset();  //v3
		  }
	  }
      // look for an ADV packet in the queue
      ieee802154e_vars.dataToSend = queue_macGetAdvPacket();
      if (ieee802154e_vars.dataToSend == NULL)
      {
        // abort the slot
        endSlot(); 
        //start outputing serial
      }
      else
      {
        // I will be sending an ADV
        // change state
        changeState(S_TXDATAOFFSET);
        // change owner
        ieee802154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
        // fill in the join info field of the ADV
        AdvInfoWriteToAdv(ieee802154e_vars.dataToSend);
        // record that I attempt to transmit this packet
        ieee802154e_vars.dataToSend->l2_numTxAttempts++;
        // arm tt1
        rtimer_schedule(DURATION_tt1);
      }
      break;
    case CELLTYPE_SYNC:   //v3  sync need update slot chain
		if (ieee802154e_isWork() == true) {
			if (idmanager_isGateway(myRole) == false) {
			    // update dedicated table slots of idle
			    ieee802154e_updateNoUseSlotInTable(&ieee802154e_advs.dSlotTable[0]);
				//update slot
				tsch_updateNodeSlot();
			}else{
				ieee802154e_updateNoUseSlotInTable(&ieee802154e_advs.dSlotTable[0]);
				tsch_updateSinkSlot();
				scheduler_push_task(checkDSlotContentionNode, TASKPRIO_RES);
			}
		}
		// abort the slot
		endSlot();
		return;
    case CELLTYPE_MON:
      // I will be listening for an ADV to monitor a neighbor
      // change state
      changeState(S_RXDATAOFFSET);
      // arm rt1
      rtimer_schedule(DURATION_rt1);
      break;
    case CELLTYPE_TXRX:
    case CELLTYPE_TX:
      // check whether we can send
      if (tsch_getOkToSend())
      {
        tsch_getNeighbor(&neighbor);
        if (tsch_isSharedSlot() == false)
        {
          if((ieee802154e_vars.slotOffset >= ONEDOWNSTARTSLOT) &&
        		  (ieee802154e_vars.slotOffset <= TWODOWNENDSLOT))
          {
        	  ieee802154e_vars.dataToSend = queue_macGetDedicatedPacket(&neighbor, NET_TYPE_CMD_CONTROL);
          }else if((ieee802154e_vars.slotOffset >= DEDICATEDSTARTSLOT) &&
        		  	  (ieee802154e_vars.slotOffset <= ONEUPENDSLOT))
          {
        	  ieee802154e_vars.dataToSend = queue_macGetDedicatedPacket(&neighbor, NET_TYPE_CAN_SAMPLE);
          }
        }
        else
        {
          if((ieee802154e_vars.slotOffset >= NOTIFYSTARTSLOT) &&
        		  (ieee802154e_vars.slotOffset <= NOTIFYENDSLOT))
          {
        	  ieee802154e_vars.dataToSend = queue_macGetSharedPacket(&neighbor, NET_TYPE_NODE_NOTIFY);
          }else if((ieee802154e_vars.slotOffset >= ONEDOWNSTARTSLOT) &&
        		        (ieee802154e_vars.slotOffset <= ONEDOWNENDSLOT))
          {
        	  if(idmanager_isGateway(myRole))
        	  {
        		  ieee802154e_vars.dataToSend = queue_macGetSharedPacket(&neighbor, NET_TYPE_CMD_CONTROL);
        	  }
          }
        }
      }
      else
      {
        ieee802154e_vars.dataToSend = NULL;
      }
      if (ieee802154e_vars.dataToSend != NULL)
      {
        // I have a packet to send
        // change state
        changeState(S_TXDATAOFFSET);
        // change owner
        ieee802154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
        // record that I attempt to transmit this packet
        ieee802154e_vars.dataToSend->l2_numTxAttempts++;
        // arm tt1
        rtimer_schedule(DURATION_tt1);
      }
      else
      {
          // change state
          changeState(S_RXDATAOFFSET);   //rev.2  to use the CELLTYPE_TXRX
          // arm rt1
          rtimer_schedule(DURATION_rt1);  //rev.2

      }
      break;
    case CELLTYPE_RX:
      // change state
      changeState(S_RXDATAOFFSET);
      // arm rt1
      rtimer_schedule(DURATION_rt1);
      break;
    case CELLTYPE_SERIALRX:
      // abort the slot
      endSlot();
#ifdef HAVE_USB
#if SET_USB_FUNC == USB_ETHERNET
      if(queue_getAvailableLength() > 4 )		 //limit serial port send speed
#elif SET_USB_FUNC == USB_VCP
	  if(queue_getAvailableLength() > 4 )		 //limit serial port send speed
      VCP_startInput();
#endif
#endif
      break;
    case CELLTYPE_CANTRX:
      // Turn off the radio
      radio_rfOff();
      // abort the slot
      endSlot();
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_IEEE802154E, ERR_WRONG_CELLTYPE,
                       (errorparameter_t)cellType,
                       (errorparameter_t)ieee802154e_vars.slotOffset);
#endif
      // abort the slot
      endSlot();
      break;
  }
}

// ====== TX
port_INLINE void activity_ti2()
{
  // change state
  changeState(S_TXDATAPREPARE);
  
  // calculate the cahnnel to transmit on
  ieee802154e_vars.channel = calculateChannel(tsch_getChannelOffset()); 
  
  // configure the radio for that channel
  radio_setChannel(ieee802154e_vars.channel);
  
  // load the packet in the radio's Tx buffer
  radio_loadPacket(ieee802154e_vars.dataToSend->payload,
                   ieee802154e_vars.dataToSend->length);
  
  // enable the radio in Tx mode. This does not send the packet.
  radio_txEnable();
  
  // arm tt2
  rtimer_schedule(DURATION_tt2);
  
  // change state
  changeState(S_TXDATAREADY);

}

port_INLINE void activity_tie1()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_MAXTXDATAPREPARE_OVERFLOW,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif
  // abort
  endSlot(); 	
}

port_INLINE void activity_ti3()
{
  // change state
  changeState(S_TXDATADELAY);
  
  // arm tt3
  rtimer_schedule(DURATION_tt3);
  
  // give the 'go' to transmit
  radio_txNow();

}

port_INLINE void activity_tie2()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_WDRADIO_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ti4(PORT_TIMER_WIDTH capturedTime)
{
  // change state
  changeState(S_TXDATA);
  
  // cancel tt3
  rtimer_cancel();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // arm tt4
  rtimer_schedule(DURATION_tt4);
}

port_INLINE void activity_tie3()
{
#if HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_WDDATADURATION_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ti5(PORT_TIMER_WIDTH capturedTime)
{

  bool listenForAck;
  
  // change state
  changeState(S_RXACKOFFSET);
  
  // cancel tt4
  rtimer_cancel();
  
  // turn off the radio
  radio_rfOff();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // decides whether to listen for an ACK
  if (packet_isBroadcastMulticast(&ieee802154e_vars.dataToSend->l2_nextORpreviousHop) == true)
  {
    listenForAck = false;
  }
  else
  {
    listenForAck = true;
  }
  
  if (listenForAck == true)
  {
    // arm tt5
    rtimer_schedule(DURATION_tt5);
  }
  else
  {
    // indicate succesful Tx to tsch to keep statistics
    //tsch_indicateTx(&ieee802154e_vars.asn, TRUE);
    // indicate to upper later the packet was sent successfully
    notif_sendDone(ieee802154e_vars.dataToSend, E_SUCCESS);
    // reset local variable
    ieee802154e_vars.dataToSend = NULL;
    // abort
    endSlot();
  }
}

port_INLINE void activity_ti6()
{

  // change state
  changeState(S_RXACKPREPARE);
  
  // calculate the channel to transmit on
  ieee802154e_vars.channel = calculateChannel(tsch_getChannelOffset()); 
  
  // configure the radio for that cahnnel
  radio_setChannel(ieee802154e_vars.channel);
  
  // enable the radio in Rx mode. The radio is not actively listening yet.
  radio_rxEnable();
  
  // arm tt6
  rtimer_schedule(DURATION_tt6);
  
  // change state
  changeState(S_RXACKREADY);
}

port_INLINE void activity_tie4()
{
#if HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_MAXRXACKPREPARE_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ti7()
{
  // change state
  changeState(S_RXACKLISTEN);
  
  // start listening
  radio_rxNow();
  
  // arm tt7
  rtimer_schedule(DURATION_tt7);
}

port_INLINE void activity_tie5()
{
  // indicate transmit failed to tsch to keep stats
  //tsch_indicateTx(&ieee802154e_vars.asn, false);
  
  // indicate transmit failed to queue to keep stats
  queue_macIndicateTx(ieee802154e_vars.dataToSend, false);
  
  //if this the can sample for the broadcast,do not re transmit

  // decrement transmits left counter
#if CAN_BROAD_SAMPLE_NO_RETRY == 1
  if((ieee802154e_vars.dataToSend->l3_payload[0] != 0x05) ||
     (ieee802154e_vars.dataToSend->l3_payload[2] != 0x01) ||
     (ieee802154e_vars.dataToSend->l3_payload[7] != 0x04) ||
     (ieee802154e_vars.dataToSend->l3_payload[8] != 0x0a))
  {
	ieee802154e_vars.dataToSend->l2_retriesLeft--;
  }
  else
  {
	//do not retramsmit the can sample for the can broadcast
	ieee802154e_vars.dataToSend->l2_retriesLeft = 0;
  }
#else
  ieee802154e_vars.dataToSend->l2_retriesLeft--;
#endif

  res_accumulateLeave(); //mj2016310
  if (ieee802154e_vars.dataToSend->l2_retriesLeft == 0)
  {
    // indicate tx fail if no more retries left
    notif_sendDone(ieee802154e_vars.dataToSend, E_FAIL);
  }
  else
  {
    // return packet to the virtual COMPONENT_RES_TO_IEEE802154E component
    ieee802154e_vars.dataToSend->owner = COMPONENT_RES_TO_IEEE802154E;
  }
  
  // reset local variable
  ieee802154e_vars.dataToSend = NULL;
  
  // abort
  endSlot();
}

port_INLINE void activity_ti8(PORT_TIMER_WIDTH capturedTime)
{
#ifdef DEBUG_TIRI
  uint16_t ti8A = rtimer_getCurrentValue();
  if(ti8A > ti.ti8A)
  {
	  ti.ti8A = ti8A;
  }
#endif

  // change state
  changeState(S_RXACK);
  
  // cancel tt7
  rtimer_cancel();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // arm tt8
  rtimer_schedule(DURATION_tt8);

#ifdef DEBUG_TIRI
  uint16_t ti8B = rtimer_getCurrentValue();
  if(ti8B > ti.ti8B)
  {
	  ti.ti8B = ti8B;
  }
#endif
}

port_INLINE void activity_tie6()
{
  // abort
  endSlot();
}

port_INLINE void activity_ti9(PORT_TIMER_WIDTH capturedTime)
{
#ifdef DEBUG_TIRI
  uint16_t ti9A = rtimer_getCurrentValue();
  if(ti9A > ti.ti9A)
  {
	  ti.ti9A = ti9A;
  }
#endif

  ieee802154_header_iht           ieee802154_header;
  volatile PORT_SIGNED_INT_WIDTH  timeCorrection;
  uint8_t                         byte0;
  uint8_t                         byte1;
  uint8_t                         myRole = idmanager_getMyRole();
  
  // change state
  changeState(S_TXPROC);
  
  // cancel tt8
  rtimer_cancel();
  
  // turn off the radio
  radio_rfOff();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // get a buffer to put the (received) ACK in
  if (queue_getAvailableLength() <= 0 ||
      (ieee802154e_vars.ackReceived = queue_getFreePacketBuffer(COMPONENT_IEEE802154E)) == NULL)
  {
#if HAVE_DEBUG
    // log the error
    debug_printError(COMPONENT_IEEE802154E, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif    
    // abort
    endSlot();
    return;
  }
  
  // declare ownership over that packet
  ieee802154e_vars.ackReceived->creator = COMPONENT_IEEE802154E;
  ieee802154e_vars.ackReceived->owner   = COMPONENT_IEEE802154E;
  
  // retrieve the received ack frame from the radio's Rx buffer
  ieee802154e_vars.ackReceived->payload = &(ieee802154e_vars.ackReceived->packet[0]);
  radio_getReceivedFrame(ieee802154e_vars.ackReceived->payload,
                         &ieee802154e_vars.ackReceived->length,
                         sizeof(ieee802154e_vars.ackReceived->packet),
                         &ieee802154e_vars.ackReceived->l1_rssi,
                         &ieee802154e_vars.ackReceived->l1_lqi,
                         &ieee802154e_vars.ackReceived->l1_crc);
  // toss CRC (2 last bytes)
  packet_tossFooter(ieee802154e_vars.ackReceived, LENGTH_CRC);
  
  /*
  The do-while loop that follows is a little parsing trick.
  Because it contains a while(0) condition, it gets executed only once.
  Below the do-while loop is some code to cleans up the ack variable.
  Anywhere in the do-while loop, a break statement can be called to jump to
  the clean up code early. If the loop ends without a break, the received
  packet was correct. If it got aborted early (through a break), the packet
  was faulty.
  */
  do
  { // this "loop" is only executed once
    // break if invalid CRC
    if (ieee802154e_vars.ackReceived->l1_crc == 0)
    {
      // break from the do-while loop and execute the clean-up code below
      break;
    }
    
    // parse the IEEE802.15.4 header
    ieee802154_retrieveHeader(ieee802154e_vars.ackReceived, &ieee802154_header);
    
    // store header details in packet buffer
    ieee802154e_vars.ackReceived->l2_frameType  = ieee802154_header.frameType;
    ieee802154e_vars.ackReceived->l2_dsn        = ieee802154_header.dsn;
    memcpy(&(ieee802154e_vars.ackReceived->l2_nextORpreviousHop), &(ieee802154_header.src), sizeof(iwsn_addr_t));
    ieee802154e_vars.ackReceived->l2_pduSpecifier = ieee802154_header.pduSpecifier;
    
    // toss the IEEE802.15.4 header
    packet_tossHeader(ieee802154e_vars.ackReceived, ieee802154_header.headerLength);
    
    // if frame is a valid ACK, handle
    if (isValidAck(&ieee802154_header,ieee802154e_vars.dataToSend) == true)
    {
      // resynchronize if I'm not a SYNCSRC and ACK from preferred parent
      if (idmanager_isSyncSrc(myRole) == false &&
          neighbors_isPreferredParent(&(ieee802154e_vars.ackReceived->l2_nextORpreviousHop)) == true)
      {
        byte0 = ieee802154e_vars.ackReceived->payload[0];
        byte1 = ieee802154e_vars.ackReceived->payload[1];
#if (__BOARD == ANTHONY)
        int16_t time;
        time = (int16_t)((uint16_t) byte1 << 8 | (uint16_t) byte0);;
        timeCorrection  = (PORT_SIGNED_INT_WIDTH)time;
#else
        timeCorrection  = (PORT_SIGNED_INT_WIDTH)((PORT_TIMER_WIDTH)byte1 << 8 | (PORT_TIMER_WIDTH) byte0);
#endif
        timeCorrection /=  US_PER_TICK;
        timeCorrection  = -timeCorrection;
        synchronizeAck(timeCorrection);
        res_resetCheckLeave();
      }
      
      // inform tsch of successful transmission
      //tsch_indicateTx(&ieee802154e_vars.asn, true);
      
      // inform upper layer
      notif_sendDone(ieee802154e_vars.dataToSend, E_SUCCESS);
      ieee802154e_vars.dataToSend = NULL;
    }else{
		ieee802154e_vars.dataToSend->l2_retriesLeft--;
		res_accumulateLeave(); //mj2016310
		if (ieee802154e_vars.dataToSend->l2_retriesLeft == 0)
		{
			// indicate tx fail if no more retries left
			notif_sendDone(ieee802154e_vars.dataToSend, E_FAIL);
		}
		else
		{
			// return packet to the virtual COMPONENT_RES_TO_IEEE802154E component
			ieee802154e_vars.dataToSend->owner = COMPONENT_RES_TO_IEEE802154E;
		}
		ieee802154e_vars.dataToSend = NULL;
    }
    
    // in any case, execute the clean-up code below
  } while (0);
  
  // free the received ack so corresponding RAM memory can be recycled
  queue_freePacketBuffer(ieee802154e_vars.ackReceived);
  
  // clear local variable
  ieee802154e_vars.ackReceived = NULL;
  
  // official end of Tx slot
  endSlot();

#ifdef DEBUG_TIRI
  uint16_t ti9B = rtimer_getCurrentValue();
  if(ti9B > ti.ti9B)
  {
	  ti.ti9B = ti9B;
  }
#endif

}

// ====== RX
port_INLINE void activity_ri2()
{
#ifdef DEBUG_TIRI
  uint16_t ri2A = rtimer_getCurrentValue();
  if(ri2A > ri.ri2A)
  {
	  ri.ri2A = ri2A;
  }
#endif

  // change state
  changeState(S_RXDATAPREPARE);
  
  // calculate the channel to transmit on
  ieee802154e_vars.channel = calculateChannel(tsch_getChannelOffset()); 
  
  // configure the radio for that channel
  radio_setChannel(ieee802154e_vars.channel);
  
  // enable the radio in Rx mode. The radio does not actively listen yet.
  radio_rxEnable();
  
  // arm rt2
  rtimer_schedule(DURATION_rt2);
  
  // change state
  changeState(S_RXDATAREADY);

#ifdef DEBUG_TIRI
  uint16_t ri2B = rtimer_getCurrentValue();
  if(ri2B > ri.ri2B)
  {
	  ri.ri2B = ri2B;
  }
#endif

}

port_INLINE void activity_rie1()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_MAXRXDATAPREPARE_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ri3()
{
#ifdef DEBUG_TIRI
  uint16_t ri3A = rtimer_getCurrentValue();
  if(ri3A > ri.ri3A)
  {
	  ri.ri3A = ri3A;
  }
#endif

  // change state
  changeState(S_RXDATALISTEN);
  
  // give the 'go' to receive
  radio_rxNow();
  
  // arm rt3
  rtimer_schedule(DURATION_rt3);

#ifdef DEBUG_TIRI
  uint16_t ri3B = rtimer_getCurrentValue();
  if(ri3B > ri.ri3B)
  {
	  ri.ri3B = ri3B;
  }
#endif

}

port_INLINE void activity_rie2()
{
  // abort
  endSlot();
}

port_INLINE void activity_ri4(PORT_TIMER_WIDTH capturedTime)
{
#ifdef DEBUG_TIRI
  uint16_t ri4A = rtimer_getCurrentValue();
  if(ri4A > ri.ri4A)
  {
	  ri.ri4A = ri4A;
  }
#endif

  // change state
  changeState(S_RXDATA);
  
  // cancel rt3
  rtimer_cancel();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // record the captured time to sync
  ieee802154e_vars.syncCapturedTime = capturedTime;
  
  // arm rt4
  rtimer_schedule(DURATION_rt4);

#ifdef DEBUG_TIRI
  uint16_t ri4B = rtimer_getCurrentValue();
  if(ri4B > ri.ri4B)
  {
	  ri.ri4B = ri4B;
  }
#endif
}

port_INLINE void activity_rie3()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_WDDATADURATION_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ri5(PORT_TIMER_WIDTH capturedTime)
{
#ifdef DEBUG_TIRI
  uint16_t ri5A = rtimer_getCurrentValue();
  if(ri5A > ri.ri5A)
  {
	  ri.ri5A = ri5A;
  }
#endif

  ieee802154_header_iht ieee802154_header;
  ieee802154e_advs_t*   adv;
  uint8_t               myRole = idmanager_getMyRole();
  
  // change state
  changeState(S_TXACKOFFSET);
  
  // cancel rt4
  rtimer_cancel();
  
  // turn off the radio
  radio_rfOff();
  
  // get a buffer to put the (received) data in
  if (queue_getAvailableLength() <= 0 ||
      (ieee802154e_vars.dataReceived = queue_getFreePacketBuffer(COMPONENT_IEEE802154E)) == NULL)
  {
#ifdef HAVE_DEBUG
    // log the error
    debug_printError(COMPONENT_IEEE802154E, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    // abort
    endSlot();
    return;
  }
  
  // declare ownership over that packet
  ieee802154e_vars.dataReceived->creator = COMPONENT_IEEE802154E;
  ieee802154e_vars.dataReceived->owner   = COMPONENT_IEEE802154E;
  
  // retrieve the received data frame from the radio's Rx buffer
  ieee802154e_vars.dataReceived->payload = &(ieee802154e_vars.dataReceived->packet[0]);
  radio_getReceivedFrame(ieee802154e_vars.dataReceived->payload,
                         &ieee802154e_vars.dataReceived->length,
                         sizeof(ieee802154e_vars.dataReceived->packet),
                         &ieee802154e_vars.dataReceived->l1_rssi,
                         &ieee802154e_vars.dataReceived->l1_lqi,
                         &ieee802154e_vars.dataReceived->l1_crc);
  // toss CRC (2 last bytes)
  packet_tossFooter(ieee802154e_vars.dataReceived, LENGTH_CRC);

  /*
  The do-while loop that follows is a little parsing trick.
  Because it contains a while(0) condition, it gets executed only once.
  The behavior is:
  - if a break occurs inside the do{} body, the error code below the loop
    gets executed. This indicates something is wrong with the packet being 
    parsed.
  - if a return occurs inside the do{} body, the error code below the loop
    does not get executed. This indicates the received packet is correct.
  */
  do
  { // this "loop" is only executed once
#if ((SET_CAN == 0) | (SET_NODE_TYPE == 1))
	  led_toggle();
#endif

    // if CRC doesn't check, stop
    if (ieee802154e_vars.dataReceived->l1_crc == 0)
    {
      // jump to the error code below this do-while loop
      break;
    }
    
    // parse the IEEE802.15.4 header
    ieee802154_retrieveHeader(ieee802154e_vars.dataReceived,&ieee802154_header);
    
    // store header details in packet buffer
    ieee802154e_vars.dataReceived->l2_frameType = ieee802154_header.frameType;
    ieee802154e_vars.dataReceived->l2_dsn       = ieee802154_header.dsn;
    memcpy(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop), &(ieee802154_header.src), sizeof(iwsn_addr_t));
    ieee802154e_vars.dataReceived->l2_pduSpecifier = ieee802154_header.pduSpecifier;
    
    // toss the IEEE802.15.4 header
    packet_tossHeader(ieee802154e_vars.dataReceived,ieee802154_header.headerLength);
    
    // if I just received a valid ADV, record the ASN, join Info and toss the payload
    if (isValidAdv(&ieee802154_header) == true)
    {
#if ((SET_CAN == 0) | (SET_NODE_TYPE == 1))
      //leds_valid_adv();
#endif
      // turn off the radio
      radio_rfOff();
      
      adv = (ieee802154e_advs_t *)&ieee802154e_vars.dataReceived->payload[0];

      if (idmanager_isGateway(myRole) == false)
      {
          //update neighbor no matter whether it's my neighbor before
          neighbors_updateNeighborFromadv(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop),
        		  	  	  	  	  	  	  adv->role,
        		  	  	  	  	  	  	  &adv->parentAddr,
        		  	  	  	  	  	  	  &adv->childAddr,
        		  	  	  	  	  	  	  ieee802154e_vars.dataReceived->l1_rssi,
        		  	  	  	  	  	  	  adv->mySlotoffset);
          ieee802154e_updateDSlotTable(&adv->dSlotTable[0]);
          if(ieee802154e_isWork() == true)
          {
        	  if(neighbors_isPreferredParent(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop)))
        	  {
        		  ieee802154e_vars.deSyncTimeout = DESYNCTIMEOUT;
        		  ieee802154e_vars.numDeSync = 0;
        		  //check asn for checking synchronizing
        		  if(asnCheckFromAdv(ieee802154e_vars.dataReceived) == false)
        			  asnStoreFromAdv(ieee802154e_vars.dataReceived);
				  // synchronize  to the sender's SYNC
				  //synchronizePacket(ieee802154e_vars.syncCapturedTime);
				  // update dedicated slot table of my parent
				  ieee802154e_updateMyparentDSlotTable(&adv->dSlotTable[0]);
        	  }
          }else{
              // synchronize (for the first time) to the sender's SYNC
              synchronizePacket(ieee802154e_vars.syncCapturedTime);
          }
      }

      if (idmanager_isGateway(myRole) == true)
      {
          //update neighbor no matter whether it's my neighbor before
    	  neighbors_sinkUpdateNeighbors(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop),
        		  	  	  	  	  	  	  adv->role,
        		  	  	  	  	  	  	  &adv->parentAddr,
        		  	  	  	  	  	  	  &adv->childAddr,
        		  	  	  	  	  	  	  ieee802154e_vars.dataReceived->l1_rssi,
        		  	  	  	  	  	  	  adv->mySlotoffset,
        		  	  	  	  	  	  	  adv->childSlotoffset);
    	  ieee802154e_updateNodeSlotInTable(SLOT_ADD, &(ieee802154e_vars.dataReceived->l2_nextORpreviousHop), adv->mySlotoffset);
    	  ieee802154e_updateNodeSlotInTable(SLOT_ADD, &adv->childAddr, adv->childSlotoffset);
      }

      // toss the ADV payload
      packet_tossHeader(ieee802154e_vars.dataReceived, ADV_PAYLOAD_LENGTH);
    }
    // record the captured time
    ieee802154e_vars.lastCapturedTime = capturedTime;
    
    // if I just received an invalid frame, stop
    if (isValidRxFrame(&ieee802154_header) == false)
    {
      // jump to the error code below this do-while loop
      break;
    }
    
    // check if ack requested
    if (ieee802154_header.ackRequested == 1 &&
        (ieee802154_header.pduSpecifier & 0x07) != MAC_PDU_PKT_TYPE_ACK &&
        (ieee802154_header.pduSpecifier & 0x07) != MAC_PDU_PKT_TYPE_ADV &&
        (ieee802154_header.pduSpecifier & 0x07) != MAC_PDU_PKT_TYPE_SYNC)
    {
      // arm rt5
      rtimer_schedule(DURATION_rt5);

#ifdef DEBUG_TIRI
  uint16_t ri5B = rtimer_getCurrentValue();
  if(ri5B > ri.ri5B)
  {
	  ri.ri5B = ri5B;
  }
#endif

    }
    else
    {
      // synchronize to the received packet iif I'm not a SYNCSRC and this is my preferred parent
      if (idmanager_isSyncSrc(myRole) == false &&
          neighbors_isPreferredParent(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop)) == true)
      {
        synchronizePacket(ieee802154e_vars.syncCapturedTime);
        ieee802154e_vars.numDeSync = 0;
        ieee802154e_vars.deSyncTimeout = DESYNCTIMEOUT;
      }
      
      // indicate reception to upper layer (no ACK asked)
      notif_receive(ieee802154e_vars.dataReceived);
      
      // reset local variable
      ieee802154e_vars.dataReceived = NULL;

      // abort
      endSlot();
    }
    
    // everything went well, return here not to execute the error code below
    return;
  } while (0);
  
  // free the (invalid) received data so RAM memory can be recycled
  queue_freePacketBuffer(ieee802154e_vars.dataReceived);
  
  // clear local variable
  ieee802154e_vars.dataReceived = NULL;
  
  // abort
  endSlot();
}

port_INLINE void activity_ri6()
{

#ifdef DEBUG_TIRI
  uint16_t ri6A = rtimer_getCurrentValue();
  if(ri6A > ri.ri6A)
  {
	  ri.ri6A = ri6A;
  }
#endif

  //  uint32_t t0 =  rtimer_getCurrentValue();
  //  t0 = t0;
  uint8_t temp_8b;
  PORT_SIGNED_INT_WIDTH timeCorrection;
  
  // change state
  changeState(S_TXACKPREPARE);

  // get a buffer to put the ack to send in
  if (queue_getAvailableLength() <= 0 ||
      (ieee802154e_vars.ackToSend = queue_getFreePacketBuffer(COMPONENT_IEEE802154E)) == NULL)
  {
#ifdef HAVE_DEBUG
    // log the error
    debug_printError(COMPONENT_IEEE802154E, ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
#endif
    // indicate we received a packet anyway (we don't want to loose any)
    notif_receive(ieee802154e_vars.dataReceived);
    // free local variable
    ieee802154e_vars.dataReceived = NULL;
    // abort
    endSlot();
    return;
  }

//  uint32_t t01 =  rtimer_getCurrentValue();
//  t01 = t01;

  // declare ownership over that packet
  ieee802154e_vars.ackToSend->creator = COMPONENT_IEEE802154E;
  ieee802154e_vars.ackToSend->owner   = COMPONENT_IEEE802154E; 
  
  // calculate the time timeCorrection (this is the time when the packet arrive w.r.t the time it should be.
  timeCorrection = (PORT_SIGNED_INT_WIDTH)((PORT_SIGNED_INT_WIDTH)ieee802154e_vars.syncCapturedTime - (PORT_SIGNED_INT_WIDTH)TsTxOffset);
  
  // add the payload to the ACK (i.e. the timeCorrection)
  packet_reserveHeaderSize(ieee802154e_vars.ackToSend, sizeof(IEEE802154E_ACK_ht));
  timeCorrection  = -timeCorrection;
  timeCorrection *= US_PER_TICK;
#if (__BOARD == ANTHONY)
  int16_t time;
  time = (int16_t) timeCorrection;
  ieee802154e_vars.ackToSend->payload[0] = (uint8_t)((((int16_t)time)     ) & 0xff);
  ieee802154e_vars.ackToSend->payload[1] = (uint8_t)((((int16_t)time) >> 8) & 0xff);
#else
  ieee802154e_vars.ackToSend->payload[0] = (uint8_t)((((PORT_TIMER_WIDTH)timeCorrection)     ) & 0xff);
  ieee802154e_vars.ackToSend->payload[1] = (uint8_t)((((PORT_TIMER_WIDTH)timeCorrection) >> 8) & 0xff);
#endif
  // prepend the IEEE802.15.4 header to the ACK
  temp_8b = 0;
  temp_8b |= (MAC_PDU_PKT_TYPE_ACK  << MAC_PDU_PKT_TYPE)  & 0x07;
  temp_8b |= (MAC_PDU_PRIORITY_DATA << MAC_PDU_PRIORITY)  & 0x30;
  ieee802154e_vars.ackToSend->l2_pduSpecifier = temp_8b;
  ieee802154e_vars.ackToSend->l2_frameType = IEEE154_TYPE_DATA;
  ieee802154e_vars.ackToSend->l2_dsn       = ieee802154e_vars.dataReceived->l2_dsn;
  ieee802154_prependHeader(ieee802154e_vars.ackToSend,
                           ieee802154e_vars.ackToSend->l2_frameType,
                           IEEE154_SEC_NO_SECURITY,
                           ieee802154e_vars.dataReceived->l2_dsn,
                           &(ieee802154e_vars.dataReceived->l2_nextORpreviousHop));

  // space for 2-byte CRC
  packet_reserveFooterSize(ieee802154e_vars.ackToSend, LENGTH_CRC);
  
  // calculate the channel to transmit on
  ieee802154e_vars.channel = calculateChannel(tsch_getChannelOffset()); 
  
  // configure the radio for that channel
  radio_setChannel(ieee802154e_vars.channel);

  // load the packet in the radio's Tx buffer
  radio_loadPacket(ieee802154e_vars.ackToSend->payload,
                   ieee802154e_vars.ackToSend->length);

  // enable the radio in Tx mode. This does not send that packet.
  radio_txEnable();

  // arm rt6
   rtimer_schedule(DURATION_rt6);
  
  // change state
  changeState(S_TXACKREADY);

#ifdef DEBUG_TIRI
  uint16_t ri6B = rtimer_getCurrentValue();
  if(ri6B > ri.ri6B)
  {
	  ri.ri6B = ri6B;
  }
#endif

}

port_INLINE void activity_rie4()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_MAXTXACKPREPARE_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ri7()
{
#ifdef DEBUG_TIRI
  uint16_t ri7A = rtimer_getCurrentValue();
  if(ri7A > ri.ri7A)
  {
	  ri.ri7A = ri7A;
  }
#endif

  // change state
  changeState(S_TXACKDELAY);
  
  // arm rt7
  rtimer_schedule(DURATION_rt7);
  
  // give the 'go' to transmit
  radio_txNow();

#ifdef DEBUG_TIRI
  uint16_t ri7B = rtimer_getCurrentValue();
  if(ri7B > ri.ri7B)
  {
	  ri.ri7B = ri7B;
  }
#endif
}

port_INLINE void activity_rie5()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_WDRADIOTX_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ri8(PORT_TIMER_WIDTH capturedTime)
{

#ifdef DEBUG_TIRI
  uint16_t ri8A = rtimer_getCurrentValue();
  if(ri8A > ri.ri8A)
  {
	  ri.ri8A = ri8A;
  }
#endif

  // change state
  changeState(S_TXACK);
  
  // cancel rt7
  rtimer_cancel();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // arm rt8
  rtimer_schedule(DURATION_rt8);

#ifdef DEBUG_TIRI
  uint16_t ri8B = rtimer_getCurrentValue();
  if(ri8B > ri.ri8B)
  {
	  ri.ri8B = ri8B;
  }
#endif
}

port_INLINE void activity_rie6()
{
#ifdef HAVE_DEBUG
  // log the error
  debug_printError(COMPONENT_IEEE802154E, ERR_WDACKDURATION_OVERFLOWS,
                   (errorparameter_t)ieee802154e_vars.state,
                   (errorparameter_t)ieee802154e_vars.slotOffset);
#endif  
  // abort
  endSlot();
}

port_INLINE void activity_ri9(PORT_TIMER_WIDTH capturedTime)
{
#ifdef DEBUG_TIRI
  uint16_t ri9A = rtimer_getCurrentValue();
  if(ri9A > ri.ri9A)
  {
	  ri.ri9A = ri9A;
  }
#endif

  uint8_t myRole = idmanager_getMyRole();
  
  // change state
  changeState(S_RXPROC);
  
  // cancel rt8
  rtimer_cancel();
  
  // record the captured time
  ieee802154e_vars.lastCapturedTime = capturedTime;
  
  // free the ack we just sent so corresponding RAM memory can be recycled
  queue_freePacketBuffer(ieee802154e_vars.ackToSend);
  
  // clear local variable
  ieee802154e_vars.ackToSend = NULL;
  
  // synchronize to the received packet
  if (idmanager_isSyncSrc(myRole) == false &&
      neighbors_isPreferredParent(&(ieee802154e_vars.dataReceived->l2_nextORpreviousHop)) == true)
  {
	ieee802154e_vars.numDeSync = 0;
	ieee802154e_vars.deSyncTimeout = DESYNCTIMEOUT;
    synchronizePacket(ieee802154e_vars.syncCapturedTime);
  }
  
  // inform upper layer of reception (after ACK sent)
  notif_receive(ieee802154e_vars.dataReceived);
  
  // clear local variable
  ieee802154e_vars.dataReceived = NULL;
  
  // official end of Rx slot
  endSlot();

#ifdef DEBUG_TIRI
  uint16_t ri9B = rtimer_getCurrentValue();
  if(ri9B > ri.ri9A)
  {
	  ri.ri9B = ri9B;
  }
#endif


}

//======= frame validity check

/**
\brief Decides whether the packet I just received is a valid ADV

\param [in] ieee802514_header IEEE802.15.4 header of the packet I just received

\returns true if packet is a valid ADV, false otherwise
*/
port_INLINE bool isValidAdv(ieee802154_header_iht* ieee802154_header)
{
  return ieee802154_header->valid == true                                                         && \
         ieee802154_header->frameType == IEEE154_TYPE_DATA                                        && \
         (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ADV                         && \
         idmanager_isSameAddress(&ieee802154_header->panid, idmanager_getMyID(ADDR_PANID))        && \
         ieee802154e_vars.dataReceived->length >= ADV_PAYLOAD_LENGTH;
}

/**
\brief Decides whether the packet I just received is a valid SYNC

\param [in] ieee802514_header IEEE802.15.4 header of the packet I just received

\returns true if packet is a valid SYNC, false otherwise
*/
port_INLINE bool isValidSync(ieee802154_header_iht* ieee802154_header)
{
  return ieee802154_header->valid == true                                                         && \
         ieee802154_header->frameType == IEEE154_TYPE_DATA                                        && \
         (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_SYNC                        && \
         idmanager_isSameAddress(&ieee802154_header->panid, idmanager_getMyID(ADDR_PANID))        && \
         ieee802154e_vars.dataReceived->length == SYNC_PAYLOAD_LENGTH;
}

/**
\brief Decides whether the packet I just received is valid received frame.

A valid Rx frame satisfies the following constraints:
- its IEEE802.15.4 header is well formatted
- its a DATA of BEACON frame (i.e. not ACK and not COMMAND)
- its sent on the same PANid as mine
- its for me (unicast or broadcast)
- 

\param [in] ieee802514_header IEEE802.15.4 header of the packet I just received

\returns true if packet is valid received frame, false otherwise
*/
port_INLINE bool isValidRxFrame(ieee802154_header_iht* ieee802154_header)
{
  return ieee802154_header->valid == true                                                         &&\
         (ieee802154_header->frameType == IEEE154_TYPE_BEACON                 ||
          ieee802154_header->frameType == IEEE154_TYPE_DATA                   ||
          ieee802154_header->frameType == IEEE154_TYPE_ACK                    ||
          ieee802154_header->frameType == IEEE154_TYPE_CMD)                                       &&\
         ((ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_DATA   ||
          (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ACK    ||
          (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ADV    ||
          (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_MON    ||
          (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_LEAVE)                     &&\
         idmanager_isSameAddress(&ieee802154_header->panid, idmanager_getMyID(ADDR_PANID))        &&\
         (idmanager_isMyAddress(&ieee802154_header->dest)                     ||
          packet_isBroadcastMulticast(&ieee802154_header->dest));
}

/**
\brief Decides whether the packet I just received is a valid ACK.

A packet is a valid ACK if it satisfies the following conditions:
- the IEEE802.15.4 header is valid
- the frame type is 'ACK'
- the sequence number in the ACK matches the sequence number of the packet sent
- the ACK contains my PANid
- the packet is unicast to me
- the packet comes from the neighbor I sent the data to

\param [in] ieee802514_header IEEE802.15.4 header of the packet I just received
\param [in] packetSent points to the packet I just sent

\returns true if packet is a valid ACK, false otherwise.
*/
port_INLINE bool isValidAck(ieee802154_header_iht* ieee802154_header,
                            QueueEntry_t*          packetSent)
{
#ifdef HAVE_CHECK_SEQ
  return ieee802154_header->valid == true                                                         && \
         ieee802154_header->frameType == IEEE154_TYPE_DATA                                        && \
         (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ACK                         && \
         ieee802154_header->dsn == packetSent->l2_dsn                                             && \
         idmanager_isSameAddress(&ieee802154_header->panid, idmanager_getMyID(ADDR_PANID))        && \
         idmanager_isMyAddress(&ieee802154_header->dest)                                          && \
         idmanager_isSameAddress(&ieee802154_header->src, &packetSent->l2_nextORpreviousHop);
#else
  // poipoi don't check for seq num
  return ieee802154_header->valid == true                                                         && \
         ieee802154_header->frameType == IEEE154_TYPE_DATA                                        && \
         (ieee802154_header->pduSpecifier & 0x07) == MAC_PDU_PKT_TYPE_ACK                         && \
         idmanager_isSameAddress(&ieee802154_header->panid, idmanager_getMyID(ADDR_PANID))        && \
         idmanager_isMyAddress(&ieee802154_header->dest)                                          && \
         idmanager_isSameAddress(&ieee802154_header->src, &packetSent->l2_nextORpreviousHop);
#endif
}

//======= ASN handling

port_INLINE void incrementAsnOffset()
{
  // increment the asn
  ieee802154e_vars.asn.bytes0and1++;
  if (ieee802154e_vars.asn.bytes0and1 == 0)
  {
    ieee802154e_vars.asn.bytes2and3++;
    if (ieee802154e_vars.asn.bytes2and3 == 0)
    {
      ieee802154e_vars.asn.byte4++;
    }
  }
  // increment the offsets
  ieee802154e_vars.slotOffset = (ieee802154e_vars.slotOffset + 1) % tsch_getFrameLength();
  ieee802154e_vars.asnOffset = (ieee802154e_vars.asnOffset + 1) % (TAIL_CHANNEL - HEAD_CHANNEL + 1);;
}

slotOffset_t calculateSlotOffset(asn_t asn)
{
  return (ieee802154e_vars.asn.byte4 * 65536 * 65536ULL +
          ieee802154e_vars.asn.bytes2and3 * 65536 +
          ieee802154e_vars.asn.bytes0and1) % tsch_getFrameLength();
}

port_INLINE void asnStoreFromAdv(QueueEntry_t* advFrame)
{
  iwsn_addr_t tempNeighbor;
  // store the ASN
  ieee802154e_vars.asn.bytes0and1 = ieee802154e_vars.dataReceived->payload[0] +
                                    256 * ieee802154e_vars.dataReceived->payload[1];
  ieee802154e_vars.asn.bytes2and3 = ieee802154e_vars.dataReceived->payload[2] +
                                    256 * ieee802154e_vars.dataReceived->payload[3];
  ieee802154e_vars.asn.byte4      = ieee802154e_vars.dataReceived->payload[4];
  
  // determine the current slotOffset
  ieee802154e_vars.slotOffset     = calculateSlotOffset(ieee802154e_vars.asn);
  
  // active the slot if iI have not active it
  memset(&tempNeighbor, 0, sizeof(iwsn_addr_t));
  tsch_syncSlotOffset(ieee802154e_vars.slotOffset);
  ieee802154e_vars.nextActiveSlotOffset = tsch_getNextActiveSlotOffset();

  // infer the current asnOffset
  ieee802154e_vars.asnOffset = ieee802154e_vars.channel - HEAD_CHANNEL - tsch_getChannelOffset();
}

port_INLINE bool asnCheckFromAdv(QueueEntry_t* advFrame) //v3
{
	uint16_t asn_byte0and1;
	if(ieee802154e_isSynch() == true)
	{
		asn_byte0and1 = ieee802154e_vars.dataReceived->payload[0] +
                256 * ieee802154e_vars.dataReceived->payload[1];
		if(ieee802154e_vars.asn.bytes0and1 != asn_byte0and1)
		{
			return false;
		}
	}
	return true;
}

// join info handling

port_INLINE void AdvInfoWriteToAdv(QueueEntry_t* advFrame)
{
	ieee802154e_updateAdvInfo();
	memcpy(&advFrame->l2_payload[0], &ieee802154e_advs, sizeof(ieee802154e_advs_t));
}

//normal node update dedicated slot table
port_INLINE void ieee802154e_updateDSlotTable(uint8_t* DSlotTable) //v3
{
	uint8_t i,temp_4b;
	uint8_t Byte;
	uint8_t* myTable = ieee802154e_advs.dSlotTable;
	for(i = 0; i < 5; i++)
	{
		Byte = myTable[i];
		temp_4b = (DSlotTable[i] >> 4) & 0x0F;
		if(temp_4b >= 0 && temp_4b <= 9)
		{
			Byte &= 0x0F;
			Byte |= ((temp_4b << 4) & 0xF0);
		}
		temp_4b = DSlotTable[i] & 0x0F;
		if(temp_4b >= 0 && temp_4b <= 9)
		{
			Byte &= 0xF0;
			Byte += temp_4b;
		}
		myTable[i] = Byte;
	}
}

port_INLINE void ieee802154e_updateMyparentDSlotTable(uint8_t* DSlotTable) //v3
{
	memcpy(ieee802154e_advs.dSlotTable, DSlotTable, SLOT_TABLE_LENGTH);
}

port_INLINE void ieee802154e_updateNoUseSlotInTable(uint8_t* DSlotTable)
{
	uint8_t i;
	uint8_t m = 0;
	iwsn_addr_t tempNeighbor;
	uint8_t channeloffset = RADIO_CHANNEL - HEAD_CHANNEL;
	nodeSlotTable_t nodeSlotTable[SLOT_SUM_NUM];
	if(slotTableShellSort(nodeSlotTable, SLOT_TABLE_LENGTH) == -1)
	{
		return;
	}
	for(i = 0; i < SLOT_SUM_NUM; i++)
	{
		if(nodeSlotTable[m].slotOffset != i)
		{
			tsch_updateActiveSlot(i + DEDICATEDSTARTSLOT, CELLTYPE_TXRX, false, channeloffset, &tempNeighbor);
		}else{
			m++;
		}
	}
}

port_INLINE void ieee802154e_selectDSlotoffset() //v3
{
	uint8_t i = 0;
	iwsn_addr_t* myAddr = idmanager_getMyID(ADDR_16B);
	slotOffset_t mySlotOffset;
	nodeSlotTable_t nodeSlotTable[SLOT_SUM_NUM];
	if(slotTableShellSort(nodeSlotTable,SLOT_SUM_NUM) == -1)
	{
		return;
	}
	if(neighbors_isParentSink())
	{
		mySlotOffset = SLOT_SUM_NUM - 1;
		i = SLOT_SUM_NUM-1;
		while(i > 1)
		{
			if((nodeSlotTable[i].slotOffset >= 0) && (nodeSlotTable[i].slotOffset < SLOT_SUM_NUM))
			{
				if(nodeSlotTable[i].slotOffset == mySlotOffset)
					mySlotOffset--;
				else
					break;
			}
			i--;
		}
		if(i >= 0)
		{
			tsch_updateMydedicatedSlotoffset(mySlotOffset);
			ieee802154e_updateNodeSlotInTable(SLOT_ADD, myAddr, mySlotOffset);
		}
	}else{
		mySlotOffset = 0;
		i = 0;
		while(i < SLOT_SUM_NUM)
		{
			if(nodeSlotTable[i].slotOffset >= 0 && nodeSlotTable[i].slotOffset < SLOT_SUM_NUM)
			{
				if(nodeSlotTable[i].slotOffset == mySlotOffset)
					mySlotOffset++;
				else
					break;
			}
			i++;
		}
		if(i < 10)
		{
			tsch_updateMydedicatedSlotoffset(mySlotOffset);
			ieee802154e_updateNodeSlotInTable(SLOT_ADD, myAddr, mySlotOffset);
		}
	}
	if(i < 0 || i >= SLOT_SUM_NUM)
	{
		tsch_updateMydedicatedSlotoffset(0xFF);
	}
}

port_INLINE void checkDSlotContentionNode()
{
	nodeSlotTable_t slotTable[10];
	uint8_t nodeAddr[5];
	uint8_t i = 1,m = 1;
	memset(nodeAddr, 0, 5);
	if(slotTableShellSort(slotTable, 5) == -1)
	{
		return;
	}
	while(i < 10)
	{
		if(slotTable[i].slotOffset >= 10)
			break;
		if(slotTable[i-1].slotOffset == slotTable[i].slotOffset)
		{
			nodeAddr[0]++;
			nodeAddr[m++] = slotTable[i].index + 1;
		}
		i++;
	}
	if(nodeAddr[0] > 0)
	{
		monitor_sendSlotCollisionNotify(nodeAddr);
	}
}

port_INLINE int8_t slotTableShellSort(nodeSlotTable_t* table, uint8_t length)  //v3
{
	int8_t ret = 0;
	int8_t i = 0, j = 0;
	nodeSlotTable_t temp;
	int8_t increment = length;
	if (table == NULL || length < 0)
	{
		ret = -1;
		return ret;
	}
	if(expandDSlotTable(ieee802154e_advs.dSlotTable, SLOT_TABLE_LENGTH, table) == -1)
	{
		ret = -1;
		return ret;
	}
	while (increment > 1)
	{
		increment /= 2;
		for (i = increment; i < length; i++)
		{
			if (table[i].slotOffset<table[i - increment].slotOffset)
			{
				temp = table[i];
				for (j = i - increment; j >= 0 && table[j].slotOffset > temp.slotOffset; j = j - increment)
					table[j + increment] = table[j];
				table[j + increment] = temp;
			}
		}
	}
	return ret;
}

port_INLINE int8_t expandDSlotTable(uint8_t* Dtable, uint8_t length, nodeSlotTable_t* result)   //v3
{
	int8_t ret = 0;
	int8_t i = 0;
	if (Dtable == NULL || result == NULL || length < 0)
	{
		ret = -1;
		return ret;
	}
	while (i < length * 2)
	{
		result[i].slotOffset = Dtable[i/2] >> 4;
		result[i].index = i;
		i++;
		result[i].slotOffset = Dtable[i/2] & 0x0F;
		result[i].index = i;
		i++;
	}
	return ret;
}


//======= synchronization

void synchronizePacket(PORT_TIMER_WIDTH timeReceived)
{
  PORT_SIGNED_INT_WIDTH  timeCorrection;
  PORT_TIMER_WIDTH newPeriod;
  PORT_TIMER_WIDTH currentValue;
  PORT_TIMER_WIDTH currentPeriod;
  
  // record the current timer value and period
  currentValue                   =  rtimer_getCurrentValue();
  currentPeriod                  =  rtimer_getPeriod();
  // calculate new period
  timeCorrection                 =  (PORT_SIGNED_INT_WIDTH)((PORT_SIGNED_INT_WIDTH)timeReceived - (PORT_SIGNED_INT_WIDTH)TsTxOffset);
  newPeriod                      =  TsSlotDuration;
  // detect whether I'm too close to the edge of the slot, in that case,
  // skip a slot and increase the temporary slot length to be 2 slots long
  if (currentValue < timeReceived || currentPeriod - currentValue < RESYNCHRONIZATIONGUARD)
  {
    newPeriod                   +=  TsSlotDuration;
    incrementAsnOffset();
  }
  newPeriod                      =  (PORT_TIMER_WIDTH)((PORT_SIGNED_INT_WIDTH)newPeriod + timeCorrection);

  rtimer_setPeriod(newPeriod);
}

void synchronizeAck(PORT_SIGNED_INT_WIDTH timeCorrection)
{
  PORT_TIMER_WIDTH newPeriod;
  PORT_TIMER_WIDTH currentPeriod;
  
  // resynchronize
  currentPeriod                  =  rtimer_getPeriod();
  newPeriod                      =  (PORT_TIMER_WIDTH)((PORT_SIGNED_INT_WIDTH)currentPeriod - timeCorrection);
  rtimer_setPeriod(newPeriod);
  ieee802154e_vars.numDeSync = 0;
  ieee802154e_vars.deSyncTimeout = DESYNCTIMEOUT;
}

void ieee802154e_changeIsSync(bool newIsSync)
{
  ieee802154e_vars.isSync = newIsSync;
  
  if (ieee802154e_isSynch() == true)
  {
#ifndef HAVE_CAN
    //leds_sync_on();
#endif
  }
  else
  {
#ifndef HAVE_CAN
    //leds_sync_off();
#endif
    // change the join state to false
    ieee802154e_changeIsWork(false);
    ieee802154e_vars.listenForAdvs = 0;
    //TODO, this is a workaround as while it is not synch the queue does not reset, so it keeps growing.
    //reset the queue to avoid filling it while it is not connected.
    queue_init();
  }
}

//======= notifying upper layer

void notif_sendDone(QueueEntry_t* packetSent, error_t error)
{
  // record the outcome of the trasmission attempt
  packetSent->l2_sendDoneError   = error;
  // record the current ASN
  memcpy(&packetSent->l2_asn, &ieee802154e_vars.asn, sizeof(asn_t));

  // associate this packet with the virtual component
  // COMPONENT_IEEE802154E_TO_RES so RES can knows it's for it
  packetSent->owner              = COMPONENT_IEEE802154E_TO_RES;
  // post RES's sendDone task
  scheduler_push_task(task_resNotifSendDone, TASKPRIO_RESNOTIF_TXDONE);

  // wake up the scheduler
  SCHEDULER_WAKEUP();
}

void notif_receive(QueueEntry_t* packetReceived)
{
  // record the current ASN
  memcpy(&packetReceived->l2_asn, &ieee802154e_vars.asn, sizeof(asn_t));
  // associate this packet with the virtual component
  // COMPONENT_IEEE802154E_TO_RES so RES can knows it's for it
  packetReceived->owner          = COMPONENT_IEEE802154E_TO_RES;
  // post RES's Receive task
  scheduler_push_task(task_resNotifReceive, TASKPRIO_RESNOTIF_RX);
  // wake up the scheduler
  SCHEDULER_WAKEUP();
}

void ieee802154e_changeIsWork(bool isWork)
{
  ieee802154e_vars.isWork = isWork;
  
  if (ieee802154e_isWork() == true)
  {
      led_on();
  }
  else
  {
	ieee802154e_vars.scanSuperframe = 0;
	tsch_updateMydedicatedSlotoffset(0xFF);
	tsch_updateChildDedicatedSlotoffset(0xFF);
	neighbors_resetVars();
    led_off();
  }
}

/**
\brief Calculates the frequency channel to transmit on, based on the 
absolute slot number and the channel offset of the requested slot.

During normal operation, the channel used is a function of the 
channelOffset indicating in the schedule tsch, and of the ASN of the
slot. This ensures channel hopping, consecutive packets sent in the same slot
in the schedule are done on a difference channel.

During development, you can force single channel operation by having this
function return a constant channel number (between 11 and 26). This allows you
to use a single-channel sniffer; but you can not schedule two links on two
different channel offsets in the same slot.

\param [in] channelOffset channel offset for the current slot

\returns The calculated frequency channel, an integer between 11 and 26.
*/
port_INLINE uint8_t calculateChannel(uint8_t channelOffset)
{
#ifdef HAVE_CHANNEL_HOPPING
  // channel hopping
  uint8_t temp = HEAD_CHANNEL + (ieee802154e_vars.asnOffset + channelOffset) % (TAIL_CHANNEL - HEAD_CHANNEL + 1);
  return temp;
#else
  // poipoi: no channel hopping
  return SYNCHRONIZING_CHANNEL;
#endif
}

/**
\brief Changes the state of the IEEE802.15.4e FSM.

Besides simply updating the state global variable,
this function toggles the FSM debug pin.

\param [in] newstate The state the IEEE802.15.4e FSM is now in.
*/
void changeState(ieee802154e_state_t newstate)
{
  // update the state
  ieee802154e_vars.state = newstate;
#ifdef HAVE_DEBUG
  // wiggle the FSM debug pin
  switch (ieee802154e_vars.state)
  {
    case S_SYNCLISTEN:
    case S_TXDATAOFFSET:
    case S_SLEEP:
    case S_RXDATAOFFSET:
    case S_SYNCRX:
    case S_SYNCPROC:
    case S_TXDATAPREPARE:
    case S_TXDATAREADY:
    case S_TXDATADELAY:
    case S_TXDATA:
    case S_RXACKOFFSET:
    case S_RXACKPREPARE:
    case S_RXACKREADY:
    case S_RXACKLISTEN: 
    case S_RXACK:
    case S_TXPROC:
    case S_RXDATAPREPARE:
    case S_RXDATAREADY:
    case S_RXDATALISTEN:
    case S_RXDATA:
    case S_TXACKOFFSET:
    case S_TXACKPREPARE:
    case S_TXACKREADY:
    case S_TXACKDELAY:
    case S_TXACK:
    case S_RXPROC:
    case S_CANTX:
    case S_CANRX:
      break;
  }
#endif
}

/**
\brief Housekeeping tasks to do at the end of each slot.

This functions is called once in each slot, when there is nothing more
to do. This might be when an error occured, or when everything went well.
This function resets the state of the FSM so it is ready for the next slot.

Note that by the time this function is called, any received packet should already
have been sent to the upper layer. Similarly, in a Tx slot, the sendDone
function should already have been done. If this is not the case, this function
will do that for you, but assume that something went wrong.
*/
void endSlot()
{
	// turn off the radio
	radio_rfOff();
	// clear any pending timer
	rtimer_cancel();

	// reset capturedTimes
	ieee802154e_vars.lastCapturedTime = 0;
	ieee802154e_vars.syncCapturedTime = 0;

	// clean up dataToSend
	if (ieee802154e_vars.dataToSend != NULL)
	{
		// if everything went well, dataToSend was set to NULL in ti9
	    // getting here means transmit failed

	    // indicate Tx fail to tsch to update stats

	    // update the packet backoff
	    queue_macIndicateTx(ieee802154e_vars.dataToSend, false);

	    //decrement transmits left counter
	    ieee802154e_vars.dataToSend->l2_retriesLeft--;

	    if (ieee802154e_vars.dataToSend->l2_retriesLeft == 0)
	    {
	      // indicate tx fail if no more retries left
	      notif_sendDone(ieee802154e_vars.dataToSend, E_FAIL);
	    }
	    else
	    {
	      // return packet to the virtual COMPONENT_RES_TO_IEEE802154E component
	      ieee802154e_vars.dataToSend->owner = COMPONENT_RES_TO_IEEE802154E;
	    }

	    // reset local variable
	    ieee802154e_vars.dataToSend = NULL;
	}

	// clean up dataReceived
	if (ieee802154e_vars.dataReceived != NULL)
	{
	    // assume something went wrong. If everything went well, dataReceived
	    // would have been set to NULL in ri9.
	    // indicate  "received packet" to upper layer since we don't want to loose packets
	    notif_receive(ieee802154e_vars.dataReceived);
	    // reset local variable
	    ieee802154e_vars.dataReceived = NULL;
	}

	// clean up ackToSend
	if (ieee802154e_vars.ackToSend != NULL)
	{
		// free ackToSend so corresponding RAM memory can be recycled
	    queue_freePacketBuffer(ieee802154e_vars.ackToSend);
	    // reset local variable
	    ieee802154e_vars.ackToSend = NULL;
	}

	// clean up ackReceived
	if (ieee802154e_vars.ackReceived != NULL)
	{
	    // free ackReceived so corresponding RAM memory can be recycled
	    queue_freePacketBuffer(ieee802154e_vars.ackReceived);
	    // reset local variable
	    ieee802154e_vars.ackReceived = NULL;
	}
	//rndis_enableRx();

	// change state
	changeState(S_SLEEP);
}

//=========================== interrupt handlers ==============================

/**
\brief Indicates a new slot has just started.

This function executes in ISR mode, when the new slot timer fires.
*/
void isr_ieee802154e_newSlot()
{

  rtimer_resetPeriod(TsSlotDuration);
  if(ieee802154e_isWork() == false)
  {
	  ieee802154e_listenForAdvs();
#if SET_NODE_TYPE
	  if(ieee802154e_isNeighborScand())
	  {
		  ieee802154e_changeIsSync(true);
		  ieee802154e_changeIsWork(true);
	  }
#endif
  }
  if (ieee802154e_isSynch() == false)
  {
    activity_synchronize_newSlot();
  }
  else
  {
    activity_ti1ORri1();
  }
}

/**
\brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/
void isr_ieee802154e_timer()
{
  switch (ieee802154e_vars.state)
  {
    case S_TXDATAOFFSET:
      activity_ti2();
      break;
    case S_TXDATAPREPARE:
      activity_tie1();
      break;
    case S_TXDATAREADY:
      activity_ti3();
      break;
    case S_TXDATADELAY:
      activity_tie2();
      break;
    case S_TXDATA:
      activity_tie3();
      break;
    case S_RXACKOFFSET:
      activity_ti6();
      break;
    case S_RXACKPREPARE:
      activity_tie4();
      break;
    case S_RXACKREADY:
      activity_ti7();
      break;
    case S_RXACKLISTEN:
      activity_tie5();
      break;
    case S_RXACK:
      activity_tie6();
      break;
    case S_RXDATAOFFSET:
      activity_ri2(); 
      break;
    case S_RXDATAPREPARE:
      activity_rie1();
      break;
    case S_RXDATAREADY:
      activity_ri3();
      break;
    case S_RXDATALISTEN:
      activity_rie2();
      break;
    case S_RXDATA:
      activity_rie3();
      break;
    case S_TXACKOFFSET: 
      activity_ri6();
      break;
    case S_TXACKPREPARE:
      activity_rie4();
      break;
    case S_TXACKREADY:
      activity_ri7();
      break;
    case S_TXACKDELAY:
      activity_rie5();
      break;
    case S_TXACK:
      activity_rie6();
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_IEEE802154E, ERR_WRONG_STATE_IN_TIMERFIRES,
                       (errorparameter_t)ieee802154e_vars.state,
                       (errorparameter_t)ieee802154e_vars.slotOffset);
#endif
      // abort
      endSlot();
      break;
  }
}

/**
\brief Indicates the radio just received the first byte of a packet.

This function executes in ISR mode.
*/
void isr_ieee802154e_startOfFrame(PORT_TIMER_WIDTH capturedTime)
{
  if (ieee802154e_vars.isSync == false)
  {
    activity_synchronize_startOfFrame(capturedTime);
  }
  else
  {
    switch (ieee802154e_vars.state)
    {
      case S_TXDATADELAY:   
        activity_ti4(capturedTime);
        break;
      case S_RXACKLISTEN:
        activity_ti8(capturedTime);
        break;
      case S_RXDATALISTEN:      
        activity_ri4(capturedTime);
        break;
      case S_TXACKDELAY:
        activity_ri8(capturedTime);
        break;
      default:
#ifdef HAVE_DEBUG
        debug_printError(COMPONENT_IEEE802154E, ERR_WRONG_STATE_IN_NEWSLOT,
                         (errorparameter_t)ieee802154e_vars.state,
                         (errorparameter_t)ieee802154e_vars.slotOffset);
#endif
        // abort
        endSlot();
        break;
    }
  }
}

/**
\brief Indicates the radio just received the last byte of a packet.

This function executes in ISR mode.
*/
void isr_ieee802154e_endOfFrame(PORT_TIMER_WIDTH capturedTime)
{
  //if(ieee802154e_vars.isJoined == true  && res_wdtEnable() == true){
//	wdt_feed();
 // }
  if (ieee802154e_vars.isSync == false) 
  {
    activity_synchronize_endOfFrame(capturedTime);
  }
  else
  {
    switch (ieee802154e_vars.state)
    {
      case S_TXDATA:
        activity_ti5(capturedTime);
        break;
      case S_RXACK:
        activity_ti9(capturedTime);
        break;
      case S_RXDATA:
        activity_ri5(capturedTime);
        break;
      case S_TXACK:
        activity_ri9(capturedTime);
        break;
      default:
#ifdef HAVE_DEBUG
        debug_printError(COMPONENT_IEEE802154E, ERR_WRONG_STATE_IN_NEWSLOT,
                         (errorparameter_t)ieee802154e_vars.state,
                         (errorparameter_t)ieee802154e_vars.slotOffset);
#endif
        // abort
        endSlot();
        break;
    }
  }
}
