/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: spi.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 9th, 2018
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "spi.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
spi_vars_t spi_vars;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================

/*
SPI1:	NSS---PA4
		SCK---PA5
		MISO---PA6
		MOSI---PA7

SPI2:	NSS---PB12
		SCK---PB13
		MISO---PB14
		MOSI---PB15
*/

void spi_init()
{
  /* SPI1 init */
  GPIO_InitTypeDef GPIO_InitStruct;
  
  __HAL_RCC_SPI1_CLK_ENABLE();
  
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin       = GPIO_PIN_5;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_4;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /* Enable SPI1 */
  __HAL_SPI_ENABLE(&hspi1);
  
  /* Set SPI1 CSS high */
  HAL_GPIO_WritePin(GPIOA ,GPIO_PIN_4 , GPIO_PIN_SET);

  /* SPI2 init */
  __HAL_RCC_SPI2_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin       = GPIO_PIN_5;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_4;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hspi2.Instance = SPI2;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Enable SPI2. */
  __HAL_SPI_ENABLE(&hspi2);

  /* Set SPI2 CSS high. */
  HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_12 , GPIO_PIN_SET);

}

void spi_txrx(uint8_t*     bufTx,
              uint8_t      lenbufTx,
              spi_return_t returnType,
              uint8_t*     bufRx,
              uint8_t      maxLenBufRx,
              spi_first_t  isFirst,
              spi_last_t   isLast) {
#ifdef SPI_IN_INTERRUPT_MODE
    // disable interrupts
#endif

    // register spi frame to send
    spi_vars.pNextTxByte      =  bufTx;
    spi_vars.numTxedBytes     =  0;
    spi_vars.txBytesLeft      =  lenbufTx;
    spi_vars.returnType       =  returnType;
    spi_vars.pNextRxByte      =  bufRx;
    spi_vars.maxRxBytes       =  maxLenBufRx;
    spi_vars.isFirst          =  isFirst;
    spi_vars.isLast           =  isLast;

    // SPI is now busy
    spi_vars.busy             =  1;


    // lower CS signal to have slave listening
    if (spi_vars.isFirst==SPI_FIRST) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
    }

#ifdef SPI_IN_INTERRUPT_MODE
    // implementation 1. use a callback function when transaction finishes

    // write first byte to TX buffer
    //SPI_I2S_SendData(SPI2,*spi_vars.pNextTxByte);


    // re-enable interrupts
    //NVIC_SETPRIMASK();
#else
    // implementation 2. busy wait for each byte to be sent
    // send all bytes
    while (spi_vars.txBytesLeft>0) {
        // write next byte to TX buffer
        //SPI1->DR = *spi_vars.pNextTxByte;
        //HAL_SPI_Transmit(&hspi1, (uint8_t*)spi_vars.pNextTxByte, 1, 1000);
        //MySPI_SendData(*spi_vars.pNextTxByte);
        // busy wait on the interrupt flag
        //while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET);
		//while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET);

        // clear the interrupt flag
        //SPI_I2S_ClearFlag(SPI2,SPI_I2S_FLAG_RXNE);
        //SPI_I2S_ClearFlag(&hspi1, SPI_I2S_FLAG_RXNE);   //注意：不清除标志可能有问题
        // save the byte just received in the RX buffer

        switch (spi_vars.returnType) {
            case SPI_FIRSTBYTE:
                if (spi_vars.numTxedBytes==0) {
                    //*spi_vars.pNextRxByte   = (uint8_t)SPI1->DR;
                    //HAL_SPI_Receive(&hspi1, (uint8_t*)spi_vars.pNextRxByte, 1, 1000);
                	*spi_vars.pNextRxByte = SPI_ReadWriteByte(*spi_vars.pNextTxByte);
                    //*spi_vars.pNextRxByte = MySPI_ReceiveData();
                }
                break;
            case SPI_BUFFER:
                //*spi_vars.pNextRxByte       = (uint8_t)SPI1->DR;
            	//HAL_SPI_Receive(&hspi1, (uint8_t*)spi_vars.pNextRxByte, 1, 1000);
            	//*spi_vars.pNextRxByte = MySPI_ReceiveData();
            	*spi_vars.pNextRxByte = SPI_ReadWriteByte(*spi_vars.pNextTxByte);
                spi_vars.pNextRxByte++;
                break;
            case SPI_LASTBYTE:
                //*spi_vars.pNextRxByte       = (uint8_t)SPI1->DR;
                //HAL_SPI_Receive(&hspi1, (uint8_t*)spi_vars.pNextRxByte, 1, 1000);
            	//*spi_vars.pNextRxByte = MySPI_ReceiveData();
            	*spi_vars.pNextRxByte = SPI_ReadWriteByte(*spi_vars.pNextTxByte);
                break;
        }
        // one byte less to go
        spi_vars.pNextTxByte++;
        spi_vars.numTxedBytes++;
        spi_vars.txBytesLeft--;
    }

    // put CS signal high to signal end of transmission to slave
    if (spi_vars.isLast==SPI_LAST) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
    }

    // SPI is not busy anymore
    spi_vars.busy             =  0;
#endif
}

/*void MySPI_SendData(uint8_t da)
{
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)==RESET);
    SPI1->DR = da;
}

uint8_t MySPI_ReceiveData(void)
{
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)==RESET);
    return (uint8_t)SPI1->DR;
}*/

uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);
	return Rxdata;    //返回收到的数据
}

int32_t spi_write_and_read(SPI_HandleTypeDef* hspi, uint8_t* buf, int32_t len)
{
	len--;
	uint8_t reg_addr = buf[0];
    for(int i=1; len>0; i++,reg_addr++,len--)
		HAL_SPI_TransmitReceive(hspi, &reg_addr, buf+i, 1, 1000);
	return 0;
}
//=========================== private =========================================

//=========================== interrupt handlers ==============================
