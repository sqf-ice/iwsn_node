/**
\brief General IWSN 02bMAClow-high 'res' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __RES_H
#define __RES_H

//=========================== include =========================================
#include <stdbool.h>
#include <stdint.h>
#include "iwsn.h"
#include "timers.h"
#include "queue.h"
#include "ieee802154.h"
#include "ieee802154e.h"
#include "network.h"

//=========================== define ==========================================

enum mac_pdu_specifier_enums {
  MAC_PDU_PKT_TYPE          = 0,
  MAC_PDU_NET_KEY           = 3,
  MAC_PDU_PRIORITY          = 4,
};

enum mac_pdu_pkt_type_enums {
  MAC_PDU_PKT_TYPE_ACK      = 0,
  MAC_PDU_PKT_TYPE_ADV      = 1,
  MAC_PDU_PKT_TYPE_MON      = 2,
  MAC_PDU_PKT_TYPE_LEAVE    = 3,
  MAC_PDU_PKT_TYPE_SYNC     = 4,
  MAC_PDU_PKT_TYPE_DATA     = 7,
};

enum mac_pdu_net_key_enums {
  MAC_PDU_NET_KEY_NO        = 0,
  MAC_PDU_NET_KEY_YES       = 1,
};

enum mac_pdu_priority_enums {
  MAC_PDU_PRIORITY_ALARM    = 0,
  MAC_PDU_PRIORITY_NORMAL   = 1,
  MAC_PDU_PRIORITY_DATA     = 2,
  MAC_PDU_PRIORITY_CMD      = 3,
};

#define MAX_LENGTH_ADV_ADD_INFO 64
#define MAX_LOSS_LEAVE_COUNTS   12

enum adv_add_info_type_enums {
  ADV_ADD_INFO_TYPE_NULL    = 0,
  ADV_ADD_INFO_TYPE_SYS     = 1,
  ADV_ADD_INFO_TYPE_CAN     = 2,
};

enum usb_connect_test_enums {
	USB_CONNECT_SINK_TO_MANAGER = 0,
	USB_CONNECT_MANAGER_TO_SINK = 1,
};

//=========================== typedef =========================================

typedef struct {
  uint8_t type;
  uint8_t length;
  uint8_t info[MAX_LENGTH_ADV_ADD_INFO];
} adv_add_info_t;

typedef struct {
  uint16_t        periodMaintenance;
  uint8_t         dsn;                  // current data sequence number
  timer_id_t      timerId;
  timer_id_t	  usb_con_timerId;
  uint8_t 		  usb_discon_num;
  uint8_t         limit_adv;
  uint8_t         limit_sync;
  uint8_t         advAddInfo;
  uint8_t 		  counter1;
  uint8_t 		  counter2;
  uint8_t 		  can_cnt;
  bool			  enable_wdt;
  bool			  usb_con_flag;
  uint16_t		  usb_checkSeq;
  uint8_t         checkLeave;
  uint16_t         checkResetSink;
} res_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void    res_init();

// from upper layer
error_t res_send(QueueEntry_t *msg);

// from lower layer
void    task_resNotifNewFrame();
void    task_resNotifSendDone();
void    task_resNotifReceive();
void usb_check();

uint8_t res_getCurDsn();
bool res_wdtEnable();

void    res_addInfoStoreFromMsg(QueueEntry_t* msg);
void    res_addInfoStoreFromAdv(QueueEntry_t* msg);
void    res_addInfoWriteToAdv(QueueEntry_t* msg);
void    adv_add_send_internal(QueueEntry_t* msg);
uint8_t adv_getUnusedLength(QueueEntry_t* msg);
void    res_accumulateLeave();
void    res_resetCheckLeave();
#endif
