/**
\brief General IWSN "blink.h"

\author Yipeng Cun <cunyipeng@bjtu.edu.cn>, Jan 2018.
*/

#ifndef __BLINK_H
#define __BLINK_H

#ifdef __cplusplus
 extern "C" {
#endif 
 
//=========================== include =========================================
#include "platform.h"
#include "scheduler.h"
#include "timers.h"
#include "led.h"

//=========================== define ==========================================

//=========================== typedef =========================================
typedef struct
{
  timer_id_t    id;
} blink_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void blink_init(void);
void blink_start(void);
void blink_start_1(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLINK_H */
