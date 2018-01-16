/*
 *	File: radio_spi.c
 *	Description:
 *		Peach-specific definition of the "radio_spi"
 *	Created on: 2013-9-18
 *	Author: 
 *		Dong Yang 		<dyang@bjtu.edu.cn>
 *		Hongchao Wang 	<hcwang@bjtu.edu.cn>
 *		Hao Zhang 		<zh1047@gmail.com>
 */

//=========================== include =========================================

#include "platform.h"
#include "radio_spi.h"


//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================
void radioIRQ_Init();
//=========================== main ============================================

//=========================== public ==========================================


/*
radio-spi 管脚
PA4  SEL
PB3  SCK
PA6 MISO
PA7 MOSI

PB0 IRQ
PB1 SLPTR
PB2 REST
*/ 
void 	radio_pins_init(){
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	//PB1 SLP_TR
	GPIO_InitStructure.Pin			= GPIO_PIN_1;
	GPIO_InitStructure.Mode	        = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //pull low
		
	//[PB3] radio /RST
	GPIO_InitStructure.Pin 			= GPIO_PIN_2;
	GPIO_InitStructure.Mode			= GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_2 , GPIO_PIN_SET);
	//[PB1] radio IRQ 
	radioIRQ_Init();
}


//[PB1] radio IRQ 管脚的初始化
void radioIRQ_Init(void)
{
	//EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	//NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOB clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable AFIO clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI1 Line to PB.01 pin */
  //GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

  /* Configure EXTI1 line */
  //EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  //EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  //EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //(&EXTI_InitStructure);

  /* Enable and set priority to EXTI1 Interrupt */
/*  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/

  /* Enable and set EXTI lines 10 to 15 Interrupt to the lowest priority */
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


//=========================== private =========================================

//=========================== interrupt handlers ==============================

