/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: platform.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
  
#include <stm32l4xx_hal.h>

#ifdef HAVE_CONFIG
#include "config.h"
#endif /* HAVE_CONFIG */

//=========================== define ==========================================
#define INTERRUPT_DECLARATION()
#define DISABLE_INTERRUPTS()            __disable_irq()
#define ENABLE_INTERRUPTS()             __enable_irq()

#define SCHEDULER_WAKEUP()
#define SCHEDULER_ENABLE_INTERRUPT()
  
/* Definitions for systicks */
#define PORT_CLOCKS_PER_SEC     1000
  
#define SYSTICK_IRQ_PRIORITY    2
  
/* Definitions for timer */
#define PORT_TIMER_WIDTH                uint16_t
#define PORT_SIGNED_INT_WIDTH           int16_t
#define PORT_MAX_NUM_TIMERS             10
#define PORT_TICS_PER_MS                10
#define PORT_MAX_TICKS_IN_SINGLE_CLOCK  ((PORT_TIMER_WIDTH)0xFFFF)
  
#define TIM2_IRQ_PRIORITY               4
#define TIM3_IRQ_PRIORITY               3
#define SPI1_IRQ_PRIORITY               1

/* Definitions for led */
#define led_t                  uint16_t

/*
#define LEDS_RED_RCC            RCC_AHBPeriph_GPIOB 
#define LEDS_RED_GPIO           GPIOB
#define LEDS_RED_PIN            GPIO_PIN_3
*/

#define LD5_RCC_ENABLE    __HAL_RCC_GPIOC_CLK_ENABLE
#define LD5_GPIO_Port  GPIOC
#define LD5_PIN  GPIO_PIN_0

#define LED2_ON         0x0001

/* Definitions for buttons */
#define BUTTON_USR_RCC                  RCC_AHBPeriph_GPIOA  
#define BUTTON_USR_GPIO                 GPIOA
#define BUTTON_USR_PIN                  GPIO_Pin_2

#define BUTTON_USR_EXTI_PORT_SRC        EXTI_PortSourceGPIOA
#define BUTTON_USR_EXTI_PIN_SRC         EXTI_PinSource2

#define BUTTON_USR_EXTI_LINE            EXTI_Line2
#define BUTTON_USR_EXTI_IRQn            EXTI2_3_IRQn

#define BUTTON_USR_IRQ_PRIORITY         1


/* Definitions for uart. */
// Definition for UARTx clock resources
#define UARTx                           UART4
#define UARTx_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()
#define UARTx_TX_RX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define UARTx_FORCE_RESET()             __HAL_RCC_UART4_FORCE_RESET()
#define UARTx_RELEASE_RESET()           __HAL_RCC_UART4_RELEASE_RESET()

// Definition for UARTx Pins
#define UARTx_TX_PIN                    GPIO_PIN_0
#define UARTx_RX_PIN                    GPIO_PIN_1
#define UARTx_TX_RX_GPIO_PORT           GPIOA
#define UARTx_BAUDRATE                  38400
#define UARTx_IRQ_PRIORITY              1

// Definition for UARTx's NVIC
#define UARTx_IRQn                      UART4_IRQn
//#define UARTx_IRQHandler              UART4_IRQHandler

/* Definitions for usart. */
/*
#define USARTx                          USART1
#define USARTx_RCC                      RCC_APB2Periph_USART1
#define USARTx_GPIO_RCC                 RCC_AHBPeriph_GPIOA
#define USARTx_TX_GPIO                  GPIOA
#define USARTx_RX_GPIO                  GPIOA
#define USARTx_TX_PIN                   GPIO_Pin_9
#define USARTx_RX_PIN                   GPIO_Pin_10
#define USARTx_BAUDRATE                 38400
#define USARTx_IRQn                     USART1_IRQn

#define USARTx_IRQ_PRIORITY             4

#define USARTx_IRQHandler               USART1_IRQHandler
*/

/* Definitions for spi. */
#define SPIx                            SPI1
#define SPIx_RCC                        RCC_APB2Periph_SPI1
#define SPIx_GPIO_RCC                   RCC_AHBPeriph_GPIOA

#define SPIx_SCK_PORT                   GPIOA
#define SPIx_SCK_PIN                    GPIO_Pin_5
#define SPIx_SCK_PIN_SRC                GPIO_PinSource5
#define SPIx_SCK_PIN_AF                 GPIO_AF_0

#define SPIx_MOSI_PORT                  GPIOA
#define SPIx_MOSI_PIN                   GPIO_Pin_7
#define SPIx_MOSI_PIN_SRC               GPIO_PinSource7
#define SPIx_MOSI_PIN_AF                GPIO_AF_0

#define SPIx_MISO_PORT                  GPIOA
#define SPIx_MISO_PIN                   GPIO_Pin_6
#define SPIx_MISO_PIN_SRC               GPIO_PinSource6
#define SPIx_MISO_PIN_AF                GPIO_AF_0

#define SPIx_CSS_PORT                   GPIOA
#define SPIx_CSS_PIN                    GPIO_Pin_4

#define port_SPIx_busy_sending()        (SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()             (SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)          SPI_SendData8((SPIx),(x))
#define port_SPIx_receive_data()        SPI_ReceiveData8(SPIx)
#define port_SPIx_disable()             SPI_Cmd(SPIx,DISABLE)
#define port_SPIx_enable()              SPI_Cmd(SPIx,ENABLE)
#define port_SPIx_set_chip_select()     GPIO_SetBits(SPIx_CSS_PORT,SPIx_CSS_PIN)
#define port_SPIx_clear_chip_select()   GPIO_ResetBits(SPIx_CSS_PORT,SPIx_CSS_PIN)

#define SPIy                            SPI2
#define SPIy_RCC                        RCC_APB1Periph_SPI2
#define SPIy_GPIO_RCC                   RCC_AHBPeriph_GPIOB

#define SPIy_SCK_PORT                   GPIOB
#define SPIy_SCK_PIN                    GPIO_Pin_13
#define SPIy_SCK_PIN_SRC                GPIO_PinSource13
#define SPIy_SCK_PIN_AF                 GPIO_AF_0

#define SPIy_MOSI_PORT                  GPIOB
#define SPIy_MOSI_PIN                   GPIO_Pin_15
#define SPIy_MOSI_PIN_SRC               GPIO_PinSource15
#define SPIy_MOSI_PIN_AF                GPIO_AF_0

#define SPIy_MISO_PORT                  GPIOB
#define SPIy_MISO_PIN                   GPIO_Pin_14
#define SPIy_MISO_PIN_SRC               GPIO_PinSource14
#define SPIy_MISO_PIN_AF                GPIO_AF_0

#define SPIy_CSS_PORT                   GPIOB
#define SPIy_CSS_PIN                    GPIO_Pin_12

#define port_SPIy_busy_sending()        (SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIy_no_data()             (SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIy_send_data(x)          SPI_SendData8((SPIy),(x))
#define port_SPIy_receive_data()        SPI_ReceiveData8(SPIy)
#define port_SPIy_disable()             SPI_Cmd(SPIy,DISABLE)
#define port_SPIy_enable()              SPI_Cmd(SPIy,ENABLE)
#define port_SPIy_set_chip_select()     GPIO_SetBits(SPIy_CSS_PORT,SPIy_CSS_PIN)
#define port_SPIy_clear_chip_select()   GPIO_ResetBits(SPIy_CSS_PORT,SPIy_CSS_PIN)


/* Definitions for DW1000. */
#define DW_PWR_GPIO_RCC                 RCC_AHBPeriph_GPIOB
#define DW_PWR_GPIO_PORT                GPIOB  
#define DW_PWR_GPIO_PIN                 GPIO_Pin_10     // PB.10

#define DW_PA_GPIO_RCC                  RCC_AHBPeriph_GPIOB
#define DW_PA_GPIO_PORT                 GPIOB  
#define DW_PA_GPIO_PIN                  GPIO_Pin_2      // PB.2

#define DW_RST_GPIO_RCC                 RCC_AHBPeriph_GPIOB
#define DW_RST_GPIO_PORT                GPIOB  
#define DW_RST_GPIO_PIN                 GPIO_Pin_11     // PB.11

#define DW_RST_EXTI_PORT_SRC            EXTI_PortSourceGPIOB
#define DW_RST_EXTI_PIN_SRC             EXTI_PinSource11

#define DW_RST_EXTI_LINE                EXTI_Line11
#define DW_RST_EXTI_IRQn                EXTI4_15_IRQn        

#define DW_RST_IRQ_PRIORITY             1

#define port_EnableDW_PWR()             GPIO_ResetBits(DW_PWR_GPIO_PORT,DW_PWR_GPIO_PIN)
#define port_DisableDW_PWR()            GPIO_SetBits(DW_PWR_GPIO_PORT,DW_PWR_GPIO_PIN)

#define port_EnableDW_PA()              GPIO_ResetBits(DW_PA_GPIO_PORT,DW_PA_GPIO_PIN)
#define port_DisableDW_PA()             GPIO_SetBits(DW_PA_GPIO_PORT,DW_PA_GPIO_PIN)

#define DW_WKP_GPIO_RCC                 RCC_AHBPeriph_GPIOB
#define DW_WKP_GPIO_PORT                GPIOB
#define DW_WKP_GPIO_PIN                 GPIO_Pin_0      // PB.0

#define DW_INT_GPIO_RCC                 RCC_AHBPeriph_GPIOB
#define DW_INT_GPIO_PORT                GPIOB
#define DW_INT_GPIO_PIN                 GPIO_Pin_1      // PB.1
  
#define DW_INT_GPIO_PORT_SRC            EXTI_PortSourceGPIOB
#define DW_INT_GPIO_PIN_SRC             EXTI_PinSource1
  
#define DW_EXTI_LINE                    EXTI_Line1
#define DW_EXTI_IRQn                    EXTI0_1_IRQn

#define DW_EXTI_IRQ_PRIORITY            1

#define port_GetEXT_IRQStatus()         EXTI_GetITEnStatus(DW_EXTI_IRQn)
#define port_DisableEXT_IRQ()           NVIC_DisableIRQ(DW_EXTI_IRQn)
#define port_EnableEXT_IRQ()            NVIC_EnableIRQ(DW_EXTI_IRQn)
#define port_CheckEXT_IRQ()             GPIO_ReadInputDataBit(DW_INT_GPIO_PORT, DW_INT_GPIO_PIN)

/* Definitons for TSCH. */
// Time-slot related
#define PORT_TsSlotDuration             10000
// Execution speed related
#define PORT_maxTxDataPrepare           1000    // 1000us (measured 584us)
#define PORT_maxRxAckPrepare            305     //  300us (measured  64us)
#define PORT_maxRxDataPrepare           400     //  400us (measured  82us)
#define PORT_maxTxAckPrepare            305     //  300us (measured 260us)
// Radio speed related
#define PORT_delayTx                    20       //    0us (measured 0us)
#define PORT_delayRx                    0       //    0us (can not measure)

#define port_INLINE                     inline

//radio IRQ
#define PORT_RADIO_IRQ_ENABLE()    HAL_NVIC_EnableIRQ(EXTI0_IRQn)
#define PORT_RADIO_IRQ_DISABLE()   HAL_NVIC_DisableIRQ(EXTI0_IRQn)

#define PORT_PIN_RADIO_SLP_TR_CNTL_HIGH()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET)
#define PORT_PIN_RADIO_SLP_TR_CNTL_LOW()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);

// radio reset line
#define PORT_PIN_RADIO_RESET_HIGH()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SET)
#define PORT_PIN_RADIO_RESET_LOW()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET)

#define PORT_RADIO_CAPTURED_TIME()	rtimer_getCapturedTime()
//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================
//ITStatus        EXTI_GetITEnStatus(uint32_t EXTI_Line);
void            __delay_cycles(unsigned long cycles);

//=========================== macros ==========================================
// Here are three instructions in one cycle
#define delay_us(x) __delay_cycles((unsigned long)(SystemCoreClock*(double)x/3000000.0))
#define delay_ms(x) __delay_cycles((unsigned long)(SystemCoreClock*(double)x/3000.0))
#define delay_s(x)  __delay_cycles((unsigned long)(SystemCoreClock*(double)x/3.0))

#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */
