/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: timers.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

#ifndef _TIMERS_H_
#define _TIMERS_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================
/* The number of timer that can run concurrently. */
#define MAX_NUM_TIMERS                  PORT_MAX_NUM_TIMERS
#define MAX_TICKS_IN_SINGLE_CLOCK       PORT_MAX_TICKS_IN_SINGLE_CLOCK
#define TOO_MANY_TIMERS_ERROR           255

#define timer_id_t                      uint8_t

//=========================== typedef =========================================
/* The timer can be periodic or oneshot */
typedef enum 
{
  TIMER_PERIODIC, 
  TIMER_ONESHOT,
} timer_type_t;

/* The time can be in tics or in ms */
typedef enum 
{
  TIME_MS, 
  TIME_TICS,
} time_type_t;

typedef void (*timers_cbt)(void);

typedef struct 
{
  uint32_t              period_ticks;       // total number of clock ticks
  PORT_TIMER_WIDTH      ticks_remaining;    // ticks remaining before elapses
  uint16_t              wraps_remaining;    // the clock register is 16 bit, and can't count beyond 32k...
                                            // so period_ticks = wraps_remaining*(32k or uint16_t)
  timer_type_t          type;               // periodic or one-shot
  bool                  isrunning;          // is running?
  timers_cbt            callback;           // function to call when elapses
  bool                  hasExpired;         // whether the callback has to be called
} timerEntry_t;

typedef struct 
{
  timerEntry_t          timersBuf[MAX_NUM_TIMERS];
  bool                  running;
  PORT_TIMER_WIDTH      currentTimeout;     // current timeout, in ticks
} timers_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void            timers_init();
void            timers_setPeriod(timer_id_t     id, 
                                 time_type_t    timetype, 
                                 uint32_t       newPeriod);
timer_id_t      timers_start(uint32_t           duration, 
                             timer_type_t       type,
                             time_type_t        timetype,
                             timers_cbt         callback);
void            timers_stop(timer_id_t id);
void            timers_restart(timer_id_t id);
void            timers_callback();
//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _TIMERS_H_ */
