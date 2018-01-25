/**
\brief General IWSN "blink.c"

\author Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>, Jan 2018.
*/

//=========================== include =========================================
#include "blink.h"


//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
blink_vars_t blink_vars;

//=========================== prototypes ======================================
void blink_timer_cb(void);
void blink_timer_fired(void);

//=========================== main ============================================

//=========================== public ==========================================
void blink_init(void)
{
  /* Initialize local variables. */
    memset(&blink_vars, 0, sizeof(blink_vars_t));
}

void blink_start(void)
{
   blink_vars.id = timers_start(2000, TIMER_PERIODIC, TIME_MS, blink_timer_cb);
}

//=========================== private =========================================
void blink_timer_cb(void)
{
    scheduler_push_task(blink_timer_fired, TASK_PRIO_NONE);
}

void blink_timer_fired(void)
{
	led_toggle();
}

//=========================== interrupt handlers ==============================
