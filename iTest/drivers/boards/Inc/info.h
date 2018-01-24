/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: info.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

#ifndef _INFO_H_
#define _INFO_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================
  
//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================
void info_getEUI64(uint8_t* addressToWrite);
void info_get16BID(uint8_t* addressToWrite);
void info_getPANID(uint8_t* addressToWrite);
void info_getMAC48(uint8_t* addressToWrite);

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _INFO_H_ */
