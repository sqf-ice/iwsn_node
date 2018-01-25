/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: uart.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>, Hongpeng Jing <16120076@bjtu.edu.cn>
 * Date: Jan 8th, 2018
 * Function: the source/header of the project
 */

#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"


//=========================== define ==========================================
#define RXBUFFERSIZE    1
#define USART_REC_LEN  	200  	//定义最大接收字节数 200
//=========================== typedef =========================================
typedef enum 
{
  UART_EVENT_THRES,
  UART_EVENT_OVERFLOW,
} uart_event_t;

typedef void (*uart_tx_cbt)();
typedef void (*uart_rx_cbt)();

typedef struct 
{
  uart_tx_cbt txCb;
  uart_rx_cbt rxCb;
} uart_vars_t;

//=========================== variables =======================================
UART_HandleTypeDef UartHandle;
uint8_t aRxBuffer[RXBUFFERSIZE];         //HAL库使用的串口接收缓冲
uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节
//=========================== prototypes ======================================
void    uart_init();

void	uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart_enableInterrupts();
void    uart_disableInterrupts();
void    uart_clearRxInterrupts();
void    uart_clearTxInterrupts();
void    uart_writeData(uint8_t byteToWrite);
uint8_t uart_readData();


uint8_t uart_tx_isr();
uint8_t uart_rx_isr();

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _UART_H_ */
