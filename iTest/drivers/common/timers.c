/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: timers.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "timers.h"
#include "ctimer.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
timers_vars_t timers_vars;

//=========================== prototypes ======================================
void timers_callback();

//=========================== main ============================================

//=========================== public ==========================================
void timers_init()
{
  uint8_t i;

  // Initialize local variables
  timers_vars.running = false;
  for (i = 0; i < MAX_NUM_TIMERS; i++) 
  {
    timers_vars.timersBuf[i].period_ticks = 0;
    timers_vars.timersBuf[i].ticks_remaining = 0;
    timers_vars.timersBuf[i].wraps_remaining = 0;
    timers_vars.timersBuf[i].type = TIMER_ONESHOT;
    timers_vars.timersBuf[i].isrunning = false;
    timers_vars.timersBuf[i].callback = NULL;
    timers_vars.timersBuf[i].hasExpired = false;
  }

  // Set callback for ctimer module
  ctimer_set_callback(timers_callback);
}

void timers_setPeriod(timer_id_t id, time_type_t timetype, uint32_t newDuration) 
{
  if (timetype == TIME_MS) 
  {
    timers_vars.timersBuf[id].period_ticks = newDuration * PORT_TICS_PER_MS;
    timers_vars.timersBuf[id].wraps_remaining = (newDuration * PORT_TICS_PER_MS / MAX_TICKS_IN_SINGLE_CLOCK); //65535=maxValue of uint16_t
  } 
  else if (timetype == TIME_TICS) 
  {
    timers_vars.timersBuf[id].period_ticks = newDuration;
    timers_vars.timersBuf[id].wraps_remaining = (newDuration / MAX_TICKS_IN_SINGLE_CLOCK); //65535=maxValue of uint16_t
  } 
  else 
  {
    while (1);	//error
  }

  // If the number of ticks falls below a 16bit value, use it, 
  // otherwise set to max 16bit value
  if (timers_vars.timersBuf[id].wraps_remaining == 0) 
  {
    if (timetype == TIME_MS) 
    {
      timers_vars.timersBuf[id].ticks_remaining = newDuration * PORT_TICS_PER_MS;
    } 
    else if (timetype == TIME_TICS) 
    {
      timers_vars.timersBuf[id].ticks_remaining = newDuration;
    } 
    else 
    {
      while (1);	//error
    }
  } 
  else 
  {
    timers_vars.timersBuf[id].ticks_remaining = MAX_TICKS_IN_SINGLE_CLOCK;
  }
}

timer_id_t timers_start(uint32_t        duration, 
                        timer_type_t    type,
                        time_type_t     timetype, 
                        timers_cbt      callback) 
{
  uint8_t id;

  // Find an unused timer
  for (id = 0; 
       id < MAX_NUM_TIMERS && timers_vars.timersBuf[id].isrunning == true; 
       id++);

  if (id < MAX_NUM_TIMERS) 
  {
    // We found an unused timer

    // Register the timer
    timers_setPeriod(id, timetype, duration);
    timers_vars.timersBuf[id].type = type;
    timers_vars.timersBuf[id].isrunning = true;
    timers_vars.timersBuf[id].callback = callback;
    timers_vars.timersBuf[id].hasExpired = false;

    // Re-schedule the running timer, if needed
    if ((timers_vars.running == false) 
        || (timers_vars.timersBuf[id].ticks_remaining < timers_vars.currentTimeout)) 
    {
      timers_vars.currentTimeout = timers_vars.timersBuf[id].ticks_remaining;
      if (timers_vars.running == false) 
      {
        ctimer_reset();
      }
      ctimer_scheduleIn(timers_vars.timersBuf[id].ticks_remaining);
    }

    timers_vars.running = true;
  } 
  else 
  {
    return TOO_MANY_TIMERS_ERROR;
  }

  return id;
}

void timers_stop(timer_id_t id) 
{
  timers_vars.timersBuf[id].isrunning = false;
}

void timers_restart(timer_id_t id) 
{
  timers_vars.timersBuf[id].isrunning = true;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
void timers_callback()
{
  timer_id_t id;
  PORT_TIMER_WIDTH min_timeout = 5;
  bool found;

  // Step 1. Identify expired timers
  for (id = 0; id < MAX_NUM_TIMERS; id++) 
  {
    if (timers_vars.timersBuf[id].isrunning == true) 
    {
      if (timers_vars.currentTimeout >= timers_vars.timersBuf[id].ticks_remaining) 
      {
        // This timer has expired
        // Check to see if we have completed the whole timer, or we're just wrapping around the max 16bit value
        if (timers_vars.timersBuf[id].wraps_remaining == 0) 
        {
          // Declare as so
          timers_vars.timersBuf[id].hasExpired = true;
        } 
        else 
        {
          timers_vars.timersBuf[id].wraps_remaining--;
          if (timers_vars.timersBuf[id].wraps_remaining == 0) 
          {
            // If we have fully wrapped around, then set the remainring ticks to the modulus of the total ticks and the max clock value
            timers_vars.timersBuf[id].ticks_remaining = 
              (timers_vars.timersBuf[id].period_ticks) % MAX_TICKS_IN_SINGLE_CLOCK;
          } 
          else 
          {
            timers_vars.timersBuf[id].ticks_remaining = MAX_TICKS_IN_SINGLE_CLOCK;
          }
        }
      } 
      else 
      {
        // This timer is not expired

        // Update its counter
        timers_vars.timersBuf[id].ticks_remaining -= timers_vars.currentTimeout;
      }
    }
  }

  // Step 2. call callbacks of expired timers
  for (id = 0; id < MAX_NUM_TIMERS; id++) 
  {
    if (timers_vars.timersBuf[id].hasExpired == true) 
    {
      // Call the callback
      timers_vars.timersBuf[id].callback();
      timers_vars.timersBuf[id].hasExpired = false;

      // Reload the timer, if applicable
      if (timers_vars.timersBuf[id].type == TIMER_PERIODIC) 
      {
        timers_vars.timersBuf[id].wraps_remaining = 
          (timers_vars.timersBuf[id].period_ticks / MAX_TICKS_IN_SINGLE_CLOCK);//65535=maxValue of uint16_t
        // If the number of ticks falls below a 16bit value, use it, otherwise set to max 16bit value
        if (timers_vars.timersBuf[id].wraps_remaining == 0) 
        {
          timers_vars.timersBuf[id].ticks_remaining = timers_vars.timersBuf[id].period_ticks;
        } 
        else 
        {
          timers_vars.timersBuf[id].ticks_remaining = MAX_TICKS_IN_SINGLE_CLOCK;
        }
      } 
      else 
      {
        timers_vars.timersBuf[id].isrunning = false;
      }
    }
  }

  // Step 3. find the minimum remaining timeout among running timers
  found = false;
  for (id = 0; id < MAX_NUM_TIMERS; id++) 
  {
    if ((timers_vars.timersBuf[id].isrunning == true)
        && (found == false || timers_vars.timersBuf[id].ticks_remaining < min_timeout)) 
    {
      min_timeout = timers_vars.timersBuf[id].ticks_remaining;
      found = true;
    }
  }

  // Step 4. schedule next timeout
  if (found == true) 
  {
    // At least one timer pending
    timers_vars.currentTimeout = min_timeout;
    ctimer_scheduleIn(timers_vars.currentTimeout);
  } 
  else 
  {
    // No more timers pending
    timers_vars.running = false;
  }
}
