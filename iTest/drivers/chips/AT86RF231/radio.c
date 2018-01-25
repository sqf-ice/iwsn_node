/*
 *	File: radio.c
 *	Description:
 *		Peach-specific definition of the "radio"
 *	Created on: 2013-7-30
 *	Author: 
 *		Dong Yang 		<dyang@bjtu.edu.cn>
 *		Hongchao Wang 	<hcwang@bjtu.edu.cn>
 *		Hao Zhang 		<zh1047@gmail.com>
 */

//=========================== include =========================================
#include "platform.h"
#include "at86rf231.h"
#include "radio_spi.h"
#include "spi.h"
#include "radio.h"
#include "rtimer.h"
#include "led.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
radio_vars_t radio_vars;

//=========================== prototypes ======================================

void    radio_spiWriteReg(uint8_t reg_addr, uint8_t reg_setting);
uint8_t radio_spiReadReg(uint8_t reg_addr);
void    radio_spiWriteSubReg(uint8_t reg_addr,
                             uint8_t mask,
                             uint8_t position,
                             uint8_t reg_setting);
uint8_t radio_spiReadSubReg(uint8_t reg_addr,
                            uint8_t mask,
                            uint8_t position);
void    radio_spiWriteTxFifo(uint8_t* bufToWrite, uint8_t lenToWrite);
void    radio_spiReadRxFifo(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                            uint8_t* pLqi);

PORT_TIMER_WIDTH radio_getCapturedTime();
void    radio_disableIRQ();
void    radio_enableIRQ();

//=========================== main ============================================
uint8_t n[3];
//=========================== public ==========================================

void radio_init()
{
	// Pins definition for radio chip.
	radio_pins_init();

	// Initialize local variables
	memset(&radio_vars, 0, sizeof(radio_vars_t));

	// Change radio state
	radio_vars.state          = RADIOSTATE_STOPPED;

	// Configure the radio
	// Turn radio off
	radio_spiWriteSubReg(SR_TRX_CMD, CMD_FORCE_TRX_OFF);

	// Turn off the radio IRQ
	radio_disableIRQ();
	// Tell radio to fire interrupt on TRX_END and RX_START
	radio_spiWriteReg(RG_IRQ_MASK, (AT_IRQ_RX_START| AT_IRQ_TRX_END));
	//radio_spiWriteReg(RG_IRQ_MASK, 0x00);

    // Deassert the interrupt pin in casqe is high
    radio_spiReadReg(RG_IRQ_STATUS);

    // Don't use ant div, only use chip ant enna
    radio_spiWriteReg(RG_ANT_DIV, 0x05);

    // Enable external PA and have the radio calculate CRC
    //radio_spiWriteReg(RG_TRX_CTRL_1, 0x80 + 0x20);
    radio_spiWriteReg(RG_TRX_CTRL_1, 0x02);

    // Set PA_BUF_LT and the radio tx power
    //radio_spiWriteReg(RG_PHY_TX_PWR, 0xC0);
    radio_spiWriteReg(RG_PHY_TX_PWR, 0x00);

	//TEST-fxh
//	radio_spiWriteReg(RG_TRX_CTRL_1, 0x02);

    // Busy wait until radio status is TRX_OFF
    while(radio_spiReadSubReg(SR_TRX_STATUS) != ST_TRX_OFF);

    // Change radio state
    radio_vars.state          = RADIOSTATE_RFOFF;
    //n[0] = radio_spiReadSubReg(SR_VERSION_NUM);
    //n[2]  = radio_spiReadReg(0x1E);
}

void radio_setStartFrameCb(radio_capture_cbt cb)
{
	radio_vars.startFrame_cb  = cb;
}

void radio_setEndFrameCb(radio_capture_cbt cb)
{
	radio_vars.endFrame_cb    = cb;
}

void radio_reset()
{
	PORT_PIN_RADIO_RESET_LOW();
	delay_us(1);
	PORT_PIN_RADIO_RESET_HIGH();
}

void radio_setChannel(uint8_t channel)
{
	// Change radio state
	radio_vars.state = RADIOSTATE_SETTING_CHANNEL;

	// Configure the radio to the right channel
	radio_spiWriteSubReg(SR_CHANNEL, channel);

	// Change radio state
	radio_vars.state = RADIOSTATE_CHANNEL_SET;
}

void radio_rfOn()
{
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

void radio_rfOff()
{
	// Change radio state
	radio_vars.state = RADIOSTATE_TURNING_OFF;

	// Disable the radio IRQ
	radio_disableIRQ();

	// Turn off the radio IRQ
	radio_spiWriteReg(RG_IRQ_MASK, 0x00);

	radio_spiReadReg(RG_TRX_STATUS);

	// Turn radio off
	radio_spiWriteSubReg(SR_TRX_CMD, CMD_FORCE_TRX_OFF);

	// Busy wait until done
	while(radio_spiReadSubReg(SR_TRX_STATUS) != ST_TRX_OFF);

#ifdef HAVE_IWSN
#if ((SET_CAN == 0) | (SET_NODE_TYPE == 1))
	//leds_radio_off();
#else
	//leds_can_radio_off();
#endif
#endif

	// Change radio state
	radio_vars.state = RADIOSTATE_RFOFF;
}

void radio_loadPacket(uint8_t* packet, uint8_t len)
{
	// Change radio state
	radio_vars.state = RADIOSTATE_LOADING_PACKET;

	// Load packet in TXFIFO
	radio_spiWriteTxFifo(packet, len);

	// Change radio state
	radio_vars.state = RADIOSTATE_PACKET_LOADED;
}

void radio_txEnable()
{
	// Change radio state
	radio_vars.state = RADIOSTATE_ENABLING_TX;

	// Enable the radio IRQ
	radio_enableIRQ();

	// Tell radio to fire interrupt on TRX_END
	radio_spiWriteReg(RG_IRQ_MASK, AT_IRQ_TRX_END);

	// Turn on radio's PLL
	radio_spiWriteSubReg(SR_TRX_CMD, CMD_PLL_ON);

	// Busy wait until done
	while(radio_spiReadSubReg(SR_TRX_STATUS) != ST_PLL_ON);

#ifdef HAVE_IWSN
#if ((SET_CAN == 0) | (SET_NODE_TYPE == 1))
	//leds_radio_on();
#else
	leds_can_radio_on();
#endif
#endif

	// Change radio state
	radio_vars.state = RADIOSTATE_TX_ENABLED;
}

void radio_txNow()
{
	PORT_TIMER_WIDTH val;

	// Change radio state
	radio_vars.state = RADIOSTATE_TRANSMITTING;

	PORT_PIN_RADIO_SLP_TR_CNTL_HIGH();
	delay_us(1);
	PORT_PIN_RADIO_SLP_TR_CNTL_LOW();

	// The AT86RF231 does not generate an interrupt when the radio transmits the
	// SFD, which messes up the MAC state machine. The danger is that, if we leave
	// this funtion like this, any radio watchdog timer will expire.
	// Instead, we cheat an mimick a start of frame event by calling
	// ieee154e_startOfFrame from here. This also means that software can never catch
	// a radio glitch by which #radio_txEnable would not be followed by a packet being
	// transmitted (I've never seen that).
	if (radio_vars.startFrame_cb != NULL)
	{
		// call the callback
		val = radio_getCapturedTime();
		radio_vars.startFrame_cb(val);
	}
}

void radio_rxEnable()
{
	// change state
	radio_vars.state = RADIOSTATE_ENABLING_RX;

	// Enable the radio IRQ
	radio_enableIRQ();

	// Tell radio to fire interrupt on TRX_END and RX_START
	radio_spiWriteReg(RG_IRQ_MASK, (AT_IRQ_TRX_END | AT_IRQ_RX_START));

	// put radio in reception mode
	radio_spiWriteReg(RG_TRX_STATE, CMD_RX_ON);

	// busy wait until radio really listening
	while(radio_spiReadSubReg(SR_TRX_STATUS) != ST_RX_ON);

#ifdef HAVE_IWSN
#if ((SET_CAN == 0) | (SET_NODE_TYPE == 1))
	//leds_radio_on();
#else
	//leds_can_radio_on();
#endif
#endif

	// change state
	radio_vars.state = RADIOSTATE_LISTENING;
}

void radio_rxNow()
{
	// nothing to do
}

void radio_getReceivedFrame(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                             int8_t* pRssi,
                            uint8_t* pLqi,
                            uint8_t* pCrc)
{
	uint8_t temp_reg_value;

	//===== crc
	temp_reg_value  = radio_spiReadReg(RG_PHY_RSSI);
	*pCrc           = (temp_reg_value & 0x80) >> 7;  // msb is whether packet passed CRC

	//===== rssi
	// as per section 8.4.3 of the AT86RF231, the RSSI is calculate as:
	// -91 + ED [dBm]
	temp_reg_value  = radio_spiReadReg(RG_PHY_ED_LEVEL);
	*pRssi          = -91 + temp_reg_value;

	//===== packet
	radio_spiReadRxFifo(pBufRead,
			            pLenRead,
			            maxBufLen,
			            pLqi);
}

//=========================== private =========================================

void radio_spiWriteReg(uint8_t reg_addr, uint8_t reg_setting)
{
	uint8_t spi_tx_buffer[2];
	uint8_t spi_rx_buffer[2];

	// Turn addess in a 'reg write' address
	spi_tx_buffer[0] = (0xC0 | reg_addr);
	spi_tx_buffer[1] = reg_setting;

	radio_spi_txrx(spi_tx_buffer,
			       sizeof(spi_tx_buffer),
			       RADIO_SPI_BUFFER,
			       (uint8_t*)spi_rx_buffer,
			       sizeof(spi_rx_buffer),
			       RADIO_SPI_FIRST,
			       RADIO_SPI_LAST);
}

uint8_t radio_spiReadReg(uint8_t reg_addr)
{
	uint8_t spi_tx_buffer[2];
	uint8_t spi_rx_buffer[2];

	// Turn addess in a 'reg read' address
	spi_tx_buffer[0] = (0x80 | reg_addr);

	// send a no_operation command just to get the reg value
	spi_tx_buffer[1] = 0x00;

	radio_spi_txrx(spi_tx_buffer,
			       sizeof(spi_tx_buffer),
			       RADIO_SPI_BUFFER,
			       (uint8_t*)spi_rx_buffer,
			       sizeof(spi_rx_buffer),
			       RADIO_SPI_FIRST,
			       RADIO_SPI_LAST);

	return spi_rx_buffer[1];
}

void radio_spiWriteSubReg(uint8_t reg_addr,
                          uint8_t mask,
                          uint8_t position,
                          uint8_t reg_setting)
{
	uint8_t register_value;

	// Read current register value and mask area outside the subregister
	register_value   = radio_spiReadReg(reg_addr);
	register_value  &= ~mask;

	// Start preparing the new subregister value, shift in place and mask
	reg_setting    <<= position;
	reg_setting     &= mask;

	reg_setting     |= register_value;  // Set the new subregister value

	// Write the modified register value
	radio_spiWriteReg(reg_addr, reg_setting);
}

uint8_t radio_spiReadSubReg(uint8_t reg_addr,
                            uint8_t mask,
                            uint8_t position)
{
	uint8_t register_value;

	// Read current register value and mask out subregister
	register_value   = radio_spiReadReg(reg_addr);
	register_value  &= mask;
	register_value >>= position; // Align subregister value

	return register_value;
}

void radio_spiWriteTxFifo(uint8_t* bufToWrite, uint8_t lenToWrite)
{
	uint8_t spi_tx_buffer[2];
	uint8_t spi_rx_buffer[1+1+127];  // 1B SPI address, 1B length, max. 127B data

	// SPI destination address for TXFIFO
	spi_tx_buffer[0] = 0x60;

	// Length byte
	spi_tx_buffer[1] = lenToWrite;
	radio_spi_txrx(spi_tx_buffer,
			       sizeof(spi_tx_buffer),
			       RADIO_SPI_BUFFER,
			       spi_rx_buffer,
			       sizeof(spi_rx_buffer),
			       RADIO_SPI_FIRST,
			       RADIO_SPI_NOTLAST);

	radio_spi_txrx(bufToWrite,
			       lenToWrite,
			       RADIO_SPI_BUFFER,
			       spi_rx_buffer,
			       sizeof(spi_rx_buffer),
			       RADIO_SPI_NOTFIRST,
			       RADIO_SPI_LAST);
}

void radio_spiReadRxFifo(uint8_t* pBufRead,
                         uint8_t* pLenRead,
                         uint8_t  maxBufLen,
                         uint8_t* pLqi)
{
	// when reading the packet over SPI from the RX buffer, you get the following:
	// - *[1B]     dummy byte because of SPI
	// - *[1B]     length byte
	// -  [0-125B] packet (excluding CRC)
	// - *[2B]     CRC
	// - *[1B]     LQI
	uint8_t spi_tx_buffer[125];
	uint8_t spi_rx_buffer[3];

	spi_tx_buffer[0] = 0x20;

	// 2 first bytes
	radio_spi_txrx(spi_tx_buffer,
			       2,
			       RADIO_SPI_BUFFER,
			       spi_rx_buffer,
			       sizeof(spi_rx_buffer),
			       RADIO_SPI_FIRST,
			       SPI_NOTLAST);

	*pLenRead  = spi_rx_buffer[1];

	if (*pLenRead > 2 && *pLenRead <= 127)
	{
		// valid length

	    //read packet
		radio_spi_txrx(spi_tx_buffer,
				       *pLenRead,
				       RADIO_SPI_BUFFER,
				       pBufRead,
				       125,
				       RADIO_SPI_NOTFIRST,
				       RADIO_SPI_NOTLAST);

	    // CRC (2B) and LQI (1B)
	    radio_spi_txrx(spi_tx_buffer,
	    		       2+1,
	    		       RADIO_SPI_BUFFER,
	    		       spi_rx_buffer,
	    		       3,
	    		       RADIO_SPI_NOTFIRST,
	    		       RADIO_SPI_LAST);

	    *pLqi   = spi_rx_buffer[2];
	}
	else
	{
		// invalid length

	    // read a just byte to close spi
	    radio_spi_txrx(spi_tx_buffer,
	    		       1,
	    		       RADIO_SPI_BUFFER,
	    		       spi_rx_buffer,
	    		       sizeof(spi_rx_buffer),
	    		       RADIO_SPI_NOTFIRST,
	    		       RADIO_SPI_LAST);
	}
}

PORT_TIMER_WIDTH radio_getCapturedTime()
{
	return PORT_RADIO_CAPTURED_TIME();
}

void radio_disableIRQ()
{
	PORT_RADIO_IRQ_DISABLE();
}

void radio_enableIRQ()
{
	PORT_RADIO_IRQ_ENABLE();
}

//=========================== interrupt handlers ==============================

uint8_t radio_isr()
{
	PORT_TIMER_WIDTH capturedTime;
	uint8_t  irq_status;

	// Capture the time
	capturedTime = radio_getCapturedTime();

	// Reading IRQ_STATUS causes radio's IRQ pin to go low
	irq_status = radio_spiReadReg(RG_IRQ_STATUS);

	// Start of frame event
	if (irq_status & AT_IRQ_RX_START)
	{
		// Change radio state
		radio_vars.state = RADIOSTATE_RECEIVING;
		if (radio_vars.startFrame_cb != NULL)
		{
			// Call the callback
			radio_vars.startFrame_cb(capturedTime);
			// Kick the OS
			return 1;
		}
	}

	// End of frame event
	if (irq_status & AT_IRQ_TRX_END)
	{
		// Change radio state
		radio_vars.state = RADIOSTATE_TXRX_DONE;
		if (radio_vars.endFrame_cb != NULL)
		{
			// Call the callback
			radio_vars.endFrame_cb(capturedTime);
			// Kick the OS
			return 1;
		}
	}

	return 0;
}

//#endif  //#if SET_RADIO_TYPE == 1 //rf231
