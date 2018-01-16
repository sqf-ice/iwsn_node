/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: config.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================

//=========================== define ==========================================
#ifndef HAVE_BUTTON
//#define HAVE_BUTTON
#endif /* HAVE_BUTTON */
  
#ifndef HAVE_UART
#define HAVE_UART
#endif /* HAVE_UART */

#ifndef HAVE_RADIO
#define HAVE_RADIO
#endif /* HAVE_RADIO */
  
#ifndef HAVE_RADIO_INTERRUPT
#define HAVE_RADIO_INTERRUPT
#endif /* HAVE_RADIO_INTERRUPT */  
  
#ifndef HAVE_RADIO_PA_LNA
#define HAVE_RADIO_PA_LNA
#endif /* HAVE_RADIO_PA_LNA */
  
#ifndef HAVE_SERIAL
//#define HAVE_SERIAL
#endif /* HAVE_SERIAL */
  
#ifndef HAVE_RTLS
#define HAVE_RTLS
#endif /* HAVE_TWR */

#ifndef HAVE_TWR
//#define HAVE_TWR
#endif /* HAVE_TWR */  
  
#ifndef DW1000_CHAN_MODE
#define DW1000_CHAN_MODE        2
#endif
  
#define SYNCHRONIZING_CHANNEL   2
  
#define SET_NODE_TYPE           0
  
//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H_ */
