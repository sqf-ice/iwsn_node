/**
\brief General IWSN 03-Network 'route' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __ROUTE_H
#define __ROUTE_H

//=========================== include =========================================

#include "stdbool.h"
#include "iwsn.h"
#include "queue.h"

//=========================== define ==========================================

#define MAXNUMROUTES    2

//=========================== typedef =========================================


//=========================== variables =======================================

//=========================== prototypes ======================================

void route_findNexthopForMsg(QueueEntry_t* msg);

#endif
