/*
 * config.h
 *
 *  Created on: 2016-3-21
 *      Author: majian
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//=========================== define ==========================================
#define SET_NODE_TYPE       2       // 0: node		1: sink_src		2: sink
#define SET_CCA             0       // 0: close		1: open
#define SET_TS_TYPE			0    	// 0  10ms	   	1: 15ms		 	2: 20ms
#define SET_NEIGHBOR		0		// 0: close     1: open
#define SET_DEBUG_TIRI		0		// 0: close		1: open
#define SET_BACKOFF			0		// 0: close     1: open
#define SET_RSSI			1		// 0: close     1: open
#define APP_TRACE
/*these are necessarily*/
#define SET_RADIO_TYPE      0       // 0: RF212		1: RF231
#define SET_AGGREGATE       0       // 0: close		1: open
#define SET_CAN				0		// 0: close     1: open   2:Simulation
#define RETRY_TIME          6       // retransmit times
#define SET_UART		    1		// 0:close		1:open
#define SET_RADIPCHIP_TYPE  1		// 0:212		1:212b
#define SET_UART_DEBUG      0       // 0:no debug	1:have debug
#define SET_TSL	            0	    // 0 :close   1:open
#define SET_ELECTRICITY     0       // 0:clise    1:open
#define SET_SAMPLING        1       // 0:clise    1:open
#define SET_SAMPLE_DEBUG    0       // 0:close    1:open
#define SET_IWSN            1       // 0:close    1:open
#define SET_BLINK           1       // 0:close    1:open
#define SET_ECHO            0       // 0:close    1:open

#if SET_CAN == 2
#define SET_CAN_TYPE        CAN_TYPE_QIEGE
/*============================================================================
//WeldType 					CAN_TYPE_QIBAO
							CAN_TYPE_SHOUGONG
							CAN_TYPE_ZYAHOO
							CAN_TYPE_MYAHOO
							CAN_TYPE_YANHOOD
							CAN_TYPE_QIEGE
==============================WeldType=======================================*/
#endif

/*the range of node address, it's the high-order of address */
#define ADDRESS_HIGH     0x20
/*optimization for can sample */
#define CAN_BROAD_SAMPLE_NO_RETRY   0 	// 0: close     1: open

#if SET_NEIGHBOR
#define ASSIGNED_PARENT	 0x21
#endif

//setting the radio chip and channel
#if SET_RADIO_TYPE == 0

#define RADIO_CHIP_AT86RF212

#if (SET_RF212 == 0)
#define RF212_780M
#define RADIO_CHANNEL 	3 // 780M(0-3)   780M-0  782M-1 784M-2 786M-3
#else
#define RF212_900M
#define RADIO_CHANNEL 	2 // 900M(1-10)
#endif

#elif SET_RADIO_TYPE == 1

#define RADIO_CHIP_AT86RF231
#define RADIO_CHANNEL 26 // channel the node listens on to synchronize

#endif

//setting the definitions of the timegroup
#if SET_NODE_TYPE == 0
#define HAVE_APP
#if (SET_CAN == 1)
#define HAVE_CAN
#endif
#elif SET_NODE_TYPE == 1
#define HAVE_USB
#endif

#ifdef HAVE_USB
#define USB_VCP			1
#define USB_ETHERNET	2
//choose the usb class functions
#define SET_USB_FUNC	USB_ETHERNET
#endif

#if (SET_UART && SET_NODE_TYPE==0)
#define HAVE_UART
#if (SET_UART_DEBUG==1)
#define HAVE_UART_DEBUG
#endif
#if SET_SAMPLING
#if SET_SAMPLE_DEBUG
#define HAVE_SAMPLE_DEBUG
#endif
#define HAVE_SAMPLING
#endif
#endif

#if SET_AGGREGATE == 1
#define HAVE_AGGREGATE
#endif

#if SET_IWSN == 1
#define HAVE_IWSN
#define HAVE_RADIO	      //this is set to the EINT
#endif

#if SET_BLINK == 1
#define HAVE_BLINK
#endif

#if SET_ECHO == 1
#define HAVE_ECHO
#define HAVE_UART
#endif

#if SET_TSL
#define HAVE_TSL
#endif


#if SET_ELECTRICITY
#define HAVE_ELECTRICITY
#endif

#if SET_RSSI
#define RSSI_THRESHOLD    -87
#endif

#define HAVE_ADV_ADDITIONAL  	//this is set to the queue_macGetAdvPacket()

#endif /* CONFIG_H_ */
