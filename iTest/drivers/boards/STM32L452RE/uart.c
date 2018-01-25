/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: uart.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>, Hongpeng Jing <16120076@bjtu.edu.cn>
 * Date: Jan 8th, 2018
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "uart.h"
//=========================== define ==========================================
uint16_t USART_RX_STA=0;                 //接收状态标记
//=========================== typedef =========================================

//=========================== variables =======================================
uart_vars_t uart_vars;
//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void uart_init()
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Initialize local variables. */
	memset(&uart_vars, 0, sizeof(uart_vars_t));

	/* Peripheral clock enable */
    UARTx_CLK_ENABLE();

	/* Enable GPIO TX/RX clock */
	UARTx_TX_RX_GPIO_CLK_ENABLE();

	/**UART4 GPIO Configuration
	PA0     ------> UART4_TX
	PA1     ------> UART4_RX
	*/
	GPIO_InitStruct.Pin = UARTx_TX_PIN|UARTx_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(UARTx_TX_RX_GPIO_PORT, &GPIO_InitStruct);

	/* UARTx Init. */
    UartHandle.Instance = UARTx;
    UartHandle.Init.BaudRate = UARTx_BAUDRATE;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

	/* UART4 interrupt Init */
	HAL_NVIC_SetPriority(UARTx_IRQn , UARTx_IRQ_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(UARTx_IRQn);

	/* Enable the USARTx Interrupt. */
	//__HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
	//__HAL_UART_ENABLE_IT(&UartHandle,UART_IT_IDLE);

	//__HAL_UART_ENABLE(&UartHandle);
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb)
{
  uart_vars.txCb = txCb;
  uart_vars.rxCb = rxCb;
}

void uart_enableInterrupts()
{
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_TC);
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
  __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TC);
}

void uart_disableInterrupts()
{
  __HAL_UART_DISABLE_IT(&UartHandle, UART_IT_TC);
  __HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);
}

void uart_clearRxInterrupts()
{
  __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);
}

void uart_clearTxInterrupts()
{
  __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TC);
}

void uart_writeData(uint8_t byteToWrite)
{
	UARTx->TDR = ((uint16_t)byteToWrite & (uint16_t)0x01FF);
	/*if(HAL_UART_Transmit_IT(&UartHandle, write, 100) != HAL_OK)
    {
	   Error_Handler();
    }
	while(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) == RESET);*/
}

uint8_t uart_readData()
{
    return (uint8_t)(UARTx->RDR & (uint16_t)0x01FF);
	/*if(HAL_UART_Receive_IT(&UartHandle, read, 100) != HAL_OK)
	{
	   Error_Handler();
	}*/
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
uint8_t uart_tx_isr()
{
  uart_clearTxInterrupts();
  if (uart_vars.txCb != NULL)
  {
    uart_vars.txCb();
    return 1;
  }
  return 0;
}

uint8_t uart_rx_isr()
{
  uart_clearRxInterrupts();
  if (uart_vars.rxCb != NULL)
  {
    uart_vars.rxCb();
    return 1;
  }
  return 0;
}
