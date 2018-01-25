/*
 * Copyright: Beijing Jiaotong University, 2018-2022.
 * Filename: led.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>
 * Date: Jan 5th, 2018
 * Function: the source/header of the project
 */

#ifndef _LEDS_H_
#define _LEDS_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================
void led_init();
void led_on();
void led_off();
void led_toggle();

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _LEDS_H_ */
