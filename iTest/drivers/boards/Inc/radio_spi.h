/*
 * radio_spi.h
 *
 *  Created on: 2013-9-18
 *      Author: wanghc
 */

#ifndef RADIO_SPI_H_
#define RADIO_SPI_H_

//=========================== include =========================================

#include "spi.h"

//=========================== define ==========================================

#define RADIO_SPI_FIRSTBYTE		SPI_FIRSTBYTE
#define RADIO_SPI_BUFFER		SPI_BUFFER
#define RADIO_SPI_LASTBYTE		SPI_LASTBYTE

#define RADIO_SPI_NOTFIRST		SPI_NOTFIRST
#define RADIO_SPI_FIRST			SPI_FIRST

#define RADIO_SPI_NOTLAST		SPI_NOTLAST
#define RADIO_SPI_LAST			SPI_LAST

#define radio_spi_return_t		spi_return_t
#define radio_spi_first_t		spi_first_t
#define radio_spi_last_t		spi_last_t

//radio pins
#define RADIO_SPI_BASE    GPIOA_BASE


//=========================== typedef =========================================

typedef struct {
	void *tx_data;				/**< Pointer to transmit data */
	uint32_t tx_cnt;			/**< Transmit counter */
	void *rx_data;				/**< Pointer to transmit data */
	uint32_t rx_cnt;			/**< Receive counter */
	uint32_t length;			/**< Length of transfer data */
	uint32_t status;			/**< Current status of SSP activity */
} radio_spi_data_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void 	radio_pins_init();

#define radio_spi_init()	spi_init()
#define radio_spi_txrx(bufTx, lenbufTx, returnType, bufRx, maxLenBufRx, isFirst, isLast) \
	spi_txrx(bufTx, lenbufTx, returnType, bufRx, maxLenBufRx, isFirst, isLast)

#endif /* RADIO_SPI_H_ */
