/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: info.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "info.h"

//=========================== define ==========================================
#define EUI64_ADDR      ((uint32_t)0x0801FF00)
#define PANID_ADDR      ((uint32_t)0x0801FF10)

//=========================== typedef =========================================

//=========================== variables =======================================
uint8_t eui64[8]={0xDE,0xCA,0x00,0x00,0x00,0x00,0x00,0x01};
uint8_t panid[2]={0x00,0x01};

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void info_getEUI64(uint8_t* addressToWrite) 
{
#ifndef INFO_USR_SECTION
  memcpy(addressToWrite, (void const*)eui64, 8);
#else
  memcpy(addressToWrite, (void const*) EUI64_ADDR, 8);
#endif /* INFO_USR_SECTION */
}

void info_get16BID(uint8_t* addressToWrite) 
{
#ifndef INFO_USR_SECTION
  memcpy(addressToWrite, (void const*)(eui64 + 6), 2);
#else
  memcpy(addressToWrite, (void const*) (EUI64_ADDR + 6), 2);
#endif /* INFO_USR_SECTION */
}

void info_getPANID(uint8_t* addressToWrite) 
{
#ifndef INFO_USR_SECTION
  memcpy(addressToWrite, (void const*)panid, 2);
#else
  memcpy(addressToWrite, (void const*) PANID_ADDR, 2);
#endif /* INFO_USR_SECTION */
}

void info_getMAC48(uint8_t* addressToWrite) 
{
#ifndef INFO_USR_SECTION
  memcpy(addressToWrite, (void const*)eui64, 3);
  memcpy((addressToWrite + 3), (void const*)(eui64 + 5), 3);
#else
  memcpy(addressToWrite, (void const*) EUI64_ADDR, 3);
  memcpy(addressToWrite, (void const*) (EUI64_ADDR + 5), 3);
#endif /* INFO_USR_SECTION */
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
