/**
\brief General IWSN Management "cmd" declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "string.h"
#include "cmd.h"
#include "res.h"
#include "idmanager.h"
//#include "can.h"
#include "ieee802154e.h"
//#include "can_trx.h"
#include "scheduler.h"
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void cmd_processUnicast(QueueEntry_t* msg);
void cmd_processMulticast(QueueEntry_t* msg);
void cmd_processBroadcast(QueueEntry_t* msg);

//void cmd_execute(uint8_t type, uint8_t length, void *data);

//=========================== main ============================================

//=========================== public ==========================================

void cmd_receive(QueueEntry_t* msg)
{
  uint8_t subType;
  
  subType = *(uint8_t *)(&msg->payload[1]);
  
  if (ieee802154e_isWork() == false)
  {
    queue_freePacketBuffer(msg);
    return;
  }
  
  switch (subType)
  {
    case CMD_CONTROL_UNICAST:
      cmd_processUnicast(msg);
      break;
    case CMD_CONTROL_MULTICAST:
      cmd_processMulticast(msg);
      break;
    case CMD_CONTROL_BROADCAST:
      cmd_processBroadcast(msg);
      break;
    default:
      queue_freePacketBuffer(msg);
      break;
  }
}

void cmd_execute(uint8_t type, uint8_t length, void *data)
{
#ifdef HAVE_CAN
  uint8_t i;
  canEntry_t *temp;
#endif  
  switch (type)
  {
#ifdef HAVE_CAN
    case CMD_CONTROL_TYPE_CAN:
    	CAN_StoreMsgFromMCU((canEntry_t *)data);
      //can_send((canEntry_t *)data);
      break;
    case CMD_CONTROL_TYPE_CAN_AGGREGATION:
      temp = (canEntry_t *)data;
      for (i = 0; i < length / sizeof(canEntry_t); i++)
      {
    	  CAN_StoreMsgFromMCU(temp);
          temp++;
      }
      break;
#endif
    default:
      break;
  }
#ifdef HAVE_CAN
  scheduler_push_task(CAN_cmd_tx,TASKPRIO_CANNOTIF_TX);
#endif
}

//=========================== private =========================================

void cmd_processUnicast(QueueEntry_t* msg)
{
  iwsn_addr_t       repAddr;
  adv_add_info_t*  ptr;
  
  repAddr.type = ADDR_16B;
  memcpy(&(repAddr.addr_16b[0]), &(msg->payload[3]), 2);
  ptr = (adv_add_info_t *)&(msg->payload[5]);
  
  if (idmanager_isMyAddress(&repAddr) == false)
  {
#ifdef HAVE_DEBUG
#endif
    queue_freePacketBuffer(msg);
    return;
  }
  
  // Execute the cmd  
  cmd_execute(ptr->type, ptr->length, ptr->info);
  
  queue_freePacketBuffer(msg);
}

void cmd_processMulticast(QueueEntry_t* msg)
{
  uint8_t           srCnt;
  iwsn_addr_t       repAddr;
  adv_add_info_t*  ptr;
  
  srCnt = msg->payload[2];
  repAddr.type = ADDR_16B;
  memcpy(&(repAddr.addr_16b[0]), &(msg->payload[3 + srCnt * 2]), 2);
  ptr = (adv_add_info_t *)&(msg->payload[3 + (srCnt + 1) * 2]);
  
  if (idmanager_isMyAddress(&repAddr) == false)
  {
#ifdef HAVE_DEBUG
#endif
    queue_freePacketBuffer(msg);
    return;
  }
  
  // Execute the cmd  
  cmd_execute(ptr->type, ptr->length, ptr->info);
  
  queue_freePacketBuffer(msg);
}

void cmd_processBroadcast(QueueEntry_t* msg)
{
  iwsn_addr_t       repAddr;
  adv_add_info_t*   ptr;
  
  repAddr.type = ADDR_16B;
  memcpy(&(repAddr.addr_16b[0]), &(msg->payload[3]), 2);
  ptr = (adv_add_info_t *)&(msg->payload[5]);
  
  if (idmanager_isMyAddress(&repAddr) == false)
  {
#ifdef HAVE_DEBUG
#endif
    queue_freePacketBuffer(msg);
    return;
  }
  
  // Execute the cmd  
  cmd_execute(ptr->type, ptr->length, ptr->info);
  
  res_addInfoStoreFromMsg(msg);  
}
//=========================== interrupt handlers ==============================
