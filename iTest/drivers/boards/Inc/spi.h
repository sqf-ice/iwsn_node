/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: spi.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"
  
//=========================== define ==========================================
#define radio_spi_txrx(bufTx, lenbufTx, returnType, bufRx, maxLenBufRx, isFirst, isLast) \
	spi_txrx(bufTx, lenbufTx, returnType, bufRx, maxLenBufRx, isFirst, isLast)
//=========================== typedef =========================================
typedef enum {
	SPI_FIRSTBYTE        = 0,
	SPI_BUFFER           = 1,
	SPI_LASTBYTE         = 2,
} spi_return_t;

typedef enum {
	SPI_NOTFIRST         = 0,
	SPI_FIRST            = 1,
} spi_first_t;

typedef enum {
	SPI_NOTLAST          = 0,
	SPI_LAST             = 1,
} spi_last_t;

typedef void (*spi_cbt)(void);

typedef struct {
	// Information about the current transaction
	uint8_t*        pNextTxByte;
	uint8_t         numTxedBytes;
	uint8_t         txBytesLeft;
	spi_return_t    returnType;
	uint8_t*        pNextRxByte;
	uint8_t         maxRxBytes;
	spi_first_t     isFirst;
	spi_last_t      isLast;
	uint8_t         busy;         // State of the module
#ifdef SPI_IN_INTERRUPT_MODE
	spi_cbt         callback;     // Callback when module done
#endif
} spi_vars_t;
//=========================== variables =======================================

//=========================== prototypes ======================================
void spi_init(void);
void spi_configFastRate(uint16_t scalingfactor);
void spi_txrx(uint8_t*     bufTx,
                 uint8_t      lenbufTx,
                 spi_return_t returnType,
                 uint8_t*     bufRx,
                 uint8_t      maxLenBufRx,
                 spi_first_t  isFirst,
                 spi_last_t   isLast);
//void MySPI_SendData(uint8_t da);
//uint8_t MySPI_ReceiveData(void);
uint8_t SPI_ReadWriteByte(uint8_t TxData);
int32_t spi_write_and_read(SPI_HandleTypeDef* hspi, uint8_t* buf, int32_t len);
//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _SPI_H_ */
