/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: board.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"
#include "ctimer.h"
#include "rtimer.h"
#include "uart.h"
#include "led.h"
#include "spi.h"
#include "radio.h"
#include "adxl372.h"
#include "radio_spi.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================
void board_init();
void board_sleep();
void board_wakeup();
void board_reset();

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H_ */
