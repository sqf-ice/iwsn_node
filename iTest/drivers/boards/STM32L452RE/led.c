/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: led.c
 * Author: Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "led.h"
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void led_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  LED2_RCC_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  
  /* Turn off LED2*/
  led_off();
}

void led_on()
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_PIN, GPIO_PIN_SET);
}

void led_off()
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_PIN, GPIO_PIN_RESET);
}

void led_toggle()
{
  led_t led_state;
  
  led_state   = HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_PIN);
  
  if (led_state)
  {
	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_PIN, GPIO_PIN_RESET);
  }
  else
  {
	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_PIN, GPIO_PIN_SET);
  }
}


//=========================== private =========================================

//=========================== interrupt handlers ==============================
