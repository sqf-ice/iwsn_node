/**
\brief General IWSN 03-Network 'monitor' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __MONITOR_H
#define __MONITOR_H

//=========================== include =========================================

#include "network.h"
#include "queue.h"
#include "timers.h"
#include <stdbool.h>
//=========================== define ==========================================

enum state_report_enums {
  SLOT_COLLISION_NOTIFY   = 0,
};

//=========================== typedef =========================================
typedef struct {
	uint32_t 		periodMaintenance;
	bool        	needMonCheck;
	iwsn_addr_t		currentMonChild;
	timer_id_t      timerId;
	uint8_t         reportTimes;
} monitor_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void monitor_init();
void monitor_reveive(QueueEntry_t *msg);	//v3
void monitor_sendSlotCollisionNotify(uint8_t* nodeAddr); //v3
void monitor_checkChild(); //v3
#endif
