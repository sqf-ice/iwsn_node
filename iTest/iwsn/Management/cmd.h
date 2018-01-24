/**
\brief General IWSN Management "cmd" declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __TEMPLATE_H
#define __TEMPLATE_H

//=========================== include =========================================

#include "network.h"
#include "queue.h"

//=========================== define ==========================================

enum cmd_control_type_enums {
  CMD_CONTROL_TYPE_NULL             = 0,
  CMD_CONTROL_TYPE_SYS              = 1,
  CMD_CONTROL_TYPE_CAN              = 2,
  CMD_CONTROL_TYPE_CAN_AGGREGATION  = 3,
};

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void          cmd_receive(QueueEntry_t* msg);
void          cmd_execute(uint8_t type, uint8_t length, void *data);

#endif