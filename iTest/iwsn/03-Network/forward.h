/**
\brief General IWSN 03-Network 'forward' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __FORWARD_H
#define __FORWARD_H

//=========================== include =========================================

#include "queue.h"
#include "network.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void forward_receive(QueueEntry_t* msg, net_header_iht net_header);

#endif