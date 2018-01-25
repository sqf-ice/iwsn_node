/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: rtimer.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 9th, 2018
 * Function: the source/header of the project
 */

#ifndef _RTIMER_H_
#define _RTIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================
enum
{
  RTIMER_NONE           = 0x00,
  RTIMER_OVERFLOW       = 0x01,
  RTIMER_COMPARE        = 0x02,
  RTIMER_CAPTURE        = 0x03,
};

//=========================== typedef =========================================
typedef void (*rtimer_compare_cbt)();
typedef void (*rtimer_capture_cbt)();

typedef struct 
{
  rtimer_compare_cbt overflow_cb;
  rtimer_compare_cbt compare_cb;
  rtimer_capture_cbt capture_cb;
  PORT_TIMER_WIDTH period;
} rtimer_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void              rtimer_init();
void              rtimer_setOverflowCb(rtimer_compare_cbt cb);
void              rtimer_setCompareCb(rtimer_compare_cbt cb);
void              rtimer_setCaptureCb(rtimer_capture_cbt cb);
void              rtimer_start(PORT_TIMER_WIDTH period);
PORT_TIMER_WIDTH  rtimer_getCurrentValue();
void              rtimer_setPeriod(PORT_TIMER_WIDTH period);
void              rtimer_resetPeriod(PORT_TIMER_WIDTH period);
PORT_TIMER_WIDTH  rtimer_getPeriod();
void              rtimer_schedule(PORT_TIMER_WIDTH offset);
void              rtimer_cancel();
PORT_TIMER_WIDTH  rtimer_getCapturedTime();
void              rtimer_isr(uint8_t irq);

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _RTIMER_H_ */
