/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: ctimer.c
 * Author: Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "ctimer.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
ctimer_vars_t ctimer_vars;
/* Timer handler declaration */
TIM_HandleTypeDef htim3;
//=========================== prototypes ======================================
void ctimer_isr (void);
//=========================== main ============================================

//=========================== public ==========================================
void ctimer_init(void)
{
	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef sConfig;

	TIM_ClockConfigTypeDef sClockSourceConfig;

	/* Prescaler declaration */
	__IO uint32_t uwPrescalerValue = 0;

	/* Initialize local variables. */
	  memset(&ctimer_vars, 0, sizeof(ctimer_vars_t));

	/* Enable TIM3 Clock. */
	  __HAL_RCC_TIM3_CLK_ENABLE();

	/* Configure TIM3:
	*  Period = 0xFFFF
    *  prescaler = 64000000/10000-1
	*  CounterMode  = upCounting mode. */
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	  uwPrescalerValue = (uint32_t)(64000000 / 10000) - 1;

	  htim3.Instance = TIM3;

	  htim3.Init.Period        = 0xFFFF;
	  htim3.Init.Prescaler     = uwPrescalerValue;
	  htim3.Init.ClockDivision = 0;
	  htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;

	  sConfig.OCMode       = TIM_OCMODE_TOGGLE;
	  sConfig.Pulse        = 0;
	  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	  if(HAL_TIM_OC_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	  {
	      Error_Handler();
	  }

/*	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	  	  _Error_Handler(__FILE__, __LINE__);
	  }*/

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	  {
	  	  _Error_Handler(__FILE__, __LINE__);
	  }

	  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	  {
	  	  _Error_Handler(__FILE__, __LINE__);
	  }

	  if(HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
	  {
	  	  /* Starting Error */
	  	  Error_Handler();
	  }

	  /* Enable TIM3. */
	  __HAL_TIM_ENABLE(&htim3);

	  HAL_NVIC_SetPriority(TIM3_IRQn, TIM3_IRQ_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void ctimer_set_callback(ctimer_cbt cb)
{  
   ctimer_vars.cb  = cb;
}

void ctimer_reset(void)
{
  /* Reset compare. */
  //TIM_SetCompare1(TIM3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  
  /* Enable compare interrupt. */
  //TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
  __HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_CC1);

  //TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  
  /* Reset timer. */
  //TIM_SetCounter(TIM3, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  
  /* Reset timer flag. */
  ctimer_vars.initiated = false;
  
  /* Reset last timer compare value. */
  ctimer_vars.last_compare_value = 0;
}

void ctimer_scheduleIn(PORT_TIMER_WIDTH delayTicks)
{
  PORT_TIMER_WIDTH newCompareValue;
  PORT_TIMER_WIDTH currentValue;
  PORT_TIMER_WIDTH temp_last_compare_value;
  
  if (!ctimer_vars.initiated)
  {
    /* Enable it if not enabled. */
    //TIM_Cmd(TIM3, ENABLE);
    __HAL_TIM_ENABLE(&htim3);
    
    /* As the timer runs forever the first time it is turned on has a weired value. */
    ctimer_vars.last_compare_value = 0;
    ctimer_vars.initiated = true;
  }
  
  temp_last_compare_value = ctimer_vars.last_compare_value;
  
  newCompareValue = ctimer_vars.last_compare_value + delayTicks;
  ctimer_vars.last_compare_value = newCompareValue;
  
  currentValue = __HAL_TIM_GET_COUNTER(&htim3);
  
  if (delayTicks < currentValue - temp_last_compare_value)
  {
    /* We're already too late, schedule the ISR right now manually. */
    /* Setting the interrupt flag triggers an interrupt. */
    ctimer_vars.last_compare_value = currentValue;
    //TIM_GenerateEvent(TIM3, TIM_EventSource_CC1);
    HAL_TIM_GenerateEvent(&htim3,TIM_EVENTSOURCE_CC1);
  }
  else
  {
    /* This is the normal case, have timer expire at newCompareValue. */
    //TIM_SetCompare1(TIM3, newCompareValue);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newCompareValue);
    //TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_CC1);
    //TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  }
}

void ctimer_cancel_schedule(void)
{
  /* Disable TIM3 Compare interrupt. */
  //TIM_SetCompare1(TIM3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  //TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
}

PORT_TIMER_WIDTH ctimer_get_currentValue(void)
{
  //return TIM_GetCounter(TIM3);
  return __HAL_TIM_GET_COUNTER(&htim3);
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
void ctimer_isr (void)
{  
  /* Call the callback. */
  if (ctimer_vars.cb != NULL)
  {
    ctimer_vars.cb();
  }
}
