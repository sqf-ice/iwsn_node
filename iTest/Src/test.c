/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: test.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "board.h"
#include "timers.h"
#include "scheduler.h"
#include "applications.h"


//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================
int main(void)
{
   /* Initialize. */
   board_init();
   timers_init();
   scheduler_init();

   /* Applications Initialize. */
   applications_init();

   /* Start. */
   applications_start();
   scheduler_start();

   return 0;
}

//=========================== public ==========================================

//=========================== private =========================================

//=========================== interrupt handlers ==============================
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
	    /* Call ISR. */
	    ctimer_isr();
	}

	if(htim->Instance==TIM2)
	{
		/* Call ISR. */
		rtimer_isr(RTIMER_COMPARE);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
	    /* Call ISR. */
	    rtimer_isr(RTIMER_OVERFLOW);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)//如果是串口4
	{
        #ifdef HAVE_UART
           uart_rx_isr();
        #endif /* HAVE_UART */
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
        #ifdef HAVE_RADIO
           radio_isr();
 	 	#endif /* HAVE_RADIO */
  }
}

//=========================== error processing ==============================
void _Error_Handler(char * file, int line)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
}
