/**
\brief General IWSN 02bMAClow-high 'aggregate' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "aggregate.h"
#include "scheduler.h"
#include "ieee802154.h"
#include "ieee802154e.h"
#include "packet.h"
#include "res.h"
#include "idmanager.h"
#ifdef HAVE_SERIAL
#include "serial.h"
#endif

#ifdef HAVE_USB
#include "usbAPI.h"
#endif
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

aggregate_vars_t aggregate_vars;

//=========================== prototypes ======================================

void    aggregate_timer_cb();
void    timers_aggregate_fired();

//=========================== main ============================================

//=========================== public ==========================================

void aggregate_init()
{
  aggregate_vars.periodMaintenance = 1000;  // fires every 1 sec on average
  
  aggregate_vars.timerId = timers_start(aggregate_vars.periodMaintenance,
                                        TIMER_PERIODIC, TIME_MS,
                                        aggregate_timer_cb);
}

error_t aggregate_send(QueueEntry_t *msg)
{
  uint8_t myRole = idmanager_getMyRole();
  
  // change owner to IEEE802154E fetches it from queue
  msg->owner        = COMPONENT_AGGREGATE;
  msg->l2_frameType = IEEE154_TYPE_DATA;
  
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
  
  // if it cannot aggregate another data or I'm a gateway, then send it out
  if (aggregate_getLeftLength(msg) < MIN_LENGTH_AGGREGATE ||
      idmanager_isGateway(myRole) == true)
  {
    aggregate_send_internal(msg);
  }
  
  return E_SUCCESS;
}

void aggregate_send_internal(QueueEntry_t* msg)
{
  uint8_t       length = aggregate_getLeftLength(msg);
  uint8_t       myRole = idmanager_getMyRole();
  iwsn_addr_t*  myAddr = idmanager_getMyID(ADDR_16B);
  
  // toss the unnecessary bytes
  packet_tossFooter(msg, length);
  
  if (idmanager_isGateway(myRole) == true)
  {
    memcpy(&msg->aggregate[0] - 2, &myAddr->addr_16b[0], 2);
#ifdef HAVE_USB
#if SET_USB_FUNC == USB_ETHERNET
    usbeth_send(&msg->aggregate[0] - 2, msg->ag_length + 2);
#elif SET_USB_FUNC == USB_VCP
    VCP_send(VCP_MSG_DATA, &msg->aggregate[0] - 2, msg->ag_length + 2);
#endif
#endif

#ifdef HAVE_SERIAL
    serial_send(SERIAL_MSG_DATA, &msg->aggregate[0] - 2, msg->ag_length + 2);
#endif
    queue_freePacketBuffer(msg);
    return;
  }
  
  // reserve space for 2-byte CRC
  packet_reserveFooterSize(msg, 2);
  
  // change owner to IEEE802154E fetches it from queue
  msg->owner        = COMPONENT_AGGREGATE_TO_IEEE802154E;
}

void aggregate_appendData(QueueEntry_t *msg, uint8_t *dataToAppend, uint8_t length, uint8_t numOfBlock)
{
  uint8_t myRole = idmanager_getMyRole();
  
  // append the data to the aggregate packet
  memcpy(&msg->aggregate[msg->ag_length], dataToAppend, length);
  
  // update the aggregate length
  msg->ag_length += length;
  
  // update the number of aggregate block
  msg->aggregate[2] += numOfBlock; 
    
  // declare ownership over that packet
  msg->owner    = COMPONENT_AGGREGATE;
  
  // if it cannot aggregate another data or I'm a gateway, then send it out
  if (aggregate_getLeftLength(msg) < MIN_LENGTH_AGGREGATE ||
      idmanager_isGateway(myRole) == true)
  {
    aggregate_send_internal(msg);
  }
}

uint8_t aggregate_getLeftLength(QueueEntry_t *msg)
{ 
  return (MAX_LENGTH_AGGREGATE - msg->ag_length);
}

//=========================== private =========================================

void aggregate_timer_cb()
{
  scheduler_push_task(timers_aggregate_fired, TASKPRIO_RES);
}

void timers_aggregate_fired()
{
}

//=========================== interrupt handlers ==============================
