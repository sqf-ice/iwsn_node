/**
\brief General IWSN "echo.c"

\author Yipeng Cun <cunyipeng@bjtu.edu.cn>, Jan 2018.
*/

//=========================== include =========================================
#include "echo.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
echo_vars_t echo_vars;
extern uint16_t USART_RX_STA;                 //接收状态标记
//=========================== prototypes ======================================
void echo_rx_isr(void);
void echo_tx_isr(void);
//=========================== main ============================================

//=========================== public ==========================================
void echo_init(void)
{  
  /* Initialize local veraibles. */
  memset(&echo_vars, 0, sizeof(echo_vars_t));
  
  /* Set the callback functions. */
  uart_setCallbacks(echo_tx_isr, echo_rx_isr);
  
  /* Enable uart interrupts. */
  //uart_enableInterrupts();
  //uart_clearTxInterrupts();
}

void echo_start(void)
{
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
void echo_rx_isr(void)
{
   if((USART_RX_STA&0x8000)==0)//接收未完成
   {
  		if(USART_RX_STA&0x4000)//接收到了0x0d
  		{
  			if(aRxBuffer[0]!=0x0a)
  				USART_RX_STA=0;//接收错误,重新开始
  			else
  				USART_RX_STA|=0x8000;	//接收完成了
  		}
  		else //还没收到0X0D
  		{
  			if(aRxBuffer[0]==0x0d)
  				USART_RX_STA|=0x4000;
  			else
  			{
  				USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0];
  				USART_RX_STA++;
  				if(USART_RX_STA>(USART_REC_LEN-1))
  					USART_RX_STA=0;//接收数据错误,重新开始接收
  			}
  		}
   }

   uint8_t len;
   if(USART_RX_STA&0x8000)
   {
        len=USART_RX_STA&0x3fff; //得到此次接收到的数据长度
      	HAL_UART_Transmit(&UartHandle, (uint8_t*)USART_RX_BUF, len, 500);	//发送接收到的数据
      	HAL_UART_Transmit(&UartHandle, (uint8_t*)"\r\n", 2, 500);
      	while(__HAL_UART_GET_FLAG(&UartHandle,UART_FLAG_TC)!=SET);		//等待发送结束
      	USART_RX_STA=0;
      	memset(USART_RX_BUF, 0, USART_REC_LEN);
    }

    uint32_t timeout=0;
    uint32_t maxDelay=0x1FFFF;

    while (HAL_UART_GetState(&UartHandle)!=HAL_UART_STATE_READY)//等待就绪
    {
    	timeout++;////超时处理
    	if(timeout>maxDelay) break;
    }
    timeout=0;
    while(HAL_UART_Receive_IT(&UartHandle,(uint8_t *)aRxBuffer, 1)!=HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
    {
    	timeout++; //超时处理
    	if(timeout>maxDelay) break;
    }
/*
    uint8_t   rxbyte;
    //Read byte just received.
    rxbyte = uart_readData();
    //Send the received byte.
    uart_writeData(rxbyte);
*/
}

void echo_tx_isr(void)
{
//  if (echo_vars.txSize > 0)
//  {
//    uart_writeByte(echo_vars.buf[echo_vars.txIdx++]);
//    echo_vars.txIdx %= ECHO_BUF_MAX;
//    echo_vars.txSize--;
//  }
//  else
//  {
//    USART_ITConfig(USARTx, USART_IT_TC, DISABLE);
//  }
}
