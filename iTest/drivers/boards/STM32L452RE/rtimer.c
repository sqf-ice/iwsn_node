/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: rtimer.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 9th, 2018
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "rtimer.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
rtimer_vars_t rtimer_vars;
/* Timer handler declaration */
TIM_HandleTypeDef htim2;

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void rtimer_init()
{
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
  
  TIM_ClockConfigTypeDef sClockSourceConfig;

  /* Prescaler declaration */
  __IO uint32_t uwPrescalerValue = 0;

  /* Initialize local variables. */
  memset(&rtimer_vars, 0, sizeof(rtimer_vars_t));
  
  /* Enable TIM2 Clock. */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* Configure TIM2:
  *  Period = 0xFFFF
  *  prescaler = 64000000/1000000-1
  *  CounterMode  = upCounting mode. */
  /* Compute the prescaler value to have TIMx counter clock equal to 1.000 MHz */

  uwPrescalerValue = (uint32_t)(72000000 / 1000000) - 1;
  
  htim2.Instance = TIM2;
  htim2.Init.Period        = 10000;
  htim2.Init.Prescaler     = uwPrescalerValue;
  htim2.Init.ClockDivision = 0;
  htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
  
  /* Configure TIM2's out compare mode:
   *  out compare mode = toggle
   *  out compare value = 0 (useless before enable compare interrupt)
   *  enable TIM2_CH1. */
  sConfig.OCMode       = TIM_OCMODE_TOGGLE;
  sConfig.Pulse        = 0;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  if(HAL_TIM_OC_ConfigChannel(&htim2, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
  	 Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
  	 _Error_Handler(__FILE__, __LINE__);
  }
  
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
  	 _Error_Handler(__FILE__, __LINE__);
  }

  if(HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
  	 /* Starting Error */
  	 Error_Handler();
  }

  //TIM_OC1PreloadConfig(TIM3, TIM_OCPRELOAD_DISABLE);

#if 0
  /* Enable GPIOs clocks. */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  /* TIM3 channel 2 pin (PB.5) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Partial Remap TIM3_CH2 pins to PB.5. */
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
  
  /* Configure TIM3's input capture mode:
   *  enable TIM3_CH2. */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM3, &TIM_ICInitStructure);
#endif
  
  /* Enable TIM2. */
  __HAL_TIM_ENABLE(&htim2);
  
  HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
  
void rtimer_setOverflowCb(rtimer_compare_cbt cb)
{
  rtimer_vars.overflow_cb = cb;
}

void rtimer_setCompareCb(rtimer_compare_cbt cb)
{
  rtimer_vars.compare_cb = cb;
}

void rtimer_setCaptureCb(rtimer_capture_cbt cb)
{
  rtimer_vars.capture_cb = cb;
}

void rtimer_start(PORT_TIMER_WIDTH period)
{
  rtimer_resetPeriod(period);
  
  __HAL_TIM_CLEAR_FLAG(&htim2,TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
}

PORT_TIMER_WIDTH rtimer_getCurrentValue()
{
  return (PORT_TIMER_WIDTH)__HAL_TIM_GET_COUNTER(&htim2);
}

void rtimer_setPeriod(PORT_TIMER_WIDTH period)
{
  rtimer_vars.period = period;
  __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)period);
}

void rtimer_resetPeriod(PORT_TIMER_WIDTH period)
{
  __HAL_TIM_DISABLE(&htim2);
  rtimer_vars.period = period;
  __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)period);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_ENABLE(&htim2);
}

PORT_TIMER_WIDTH rtimer_getPeriod()
{
  return (PORT_TIMER_WIDTH)rtimer_vars.period;
}

void rtimer_schedule(PORT_TIMER_WIDTH offset)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, offset);
  __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
}

void rtimer_cancel()
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
}

PORT_TIMER_WIDTH rtimer_getCapturedTime()
{
  return (PORT_TIMER_WIDTH)__HAL_TIM_GET_COMPARE(&htim2, TIM_IT_CC2);
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
void rtimer_isr(uint8_t irq)
{
  if (irq == RTIMER_OVERFLOW)
  {
    if (rtimer_vars.overflow_cb != NULL)
    {
      // Call the callback
      rtimer_vars.overflow_cb();
      // Kick the OS
      return;
    }
  }
  else if (irq == RTIMER_COMPARE)
  {
    if (rtimer_vars.compare_cb != NULL)
    {
      // Call the callback
      rtimer_vars.compare_cb();
      // Kick the OS
      return;
    }
  }
  else if (irq == RTIMER_CAPTURE)
  {
    if (rtimer_vars.capture_cb != NULL)
    {
      // Call the callback
      rtimer_vars.capture_cb();
      // Kick the OS
      return;
    }
  }
  else
  {
    return;
  }
}
