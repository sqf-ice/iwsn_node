/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: ctimer.h
 * Author: Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

#ifndef _CTIMER_H_
#define _CTIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================
#define PORT_TIMER_WIDTH                uint16_t
//=========================== typedef =========================================
typedef void (*ctimer_cbt)(void);

typedef struct 
{
  ctimer_cbt            cb;
  PORT_TIMER_WIDTH      last_compare_value;
  bool                  initiated;
} ctimer_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void              ctimer_init();
void              ctimer_set_callback(ctimer_cbt cb);
void              ctimer_reset();
void              ctimer_scheduleIn(PORT_TIMER_WIDTH delayTicks);
void              ctimer_cancel_schedule();
PORT_TIMER_WIDTH  ctimer_get_currentValue();
void              ctimer_isr (void);

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _CTIMER_H_ */
