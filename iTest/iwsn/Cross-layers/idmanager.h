/**
\brief General IWSN Cross-layer 'idmanager' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __IDMANAGER_H
#define __IDMANAGER_H

//=========================== include =========================================
#include "iwsn.h"
#include "platform.h"
#include "stdbool.h"
//=========================== define ==========================================

/*
IWSN_NODE_TYPE:
        NODE ordinary         0
        SINK with SYNCSRC     1
        SINK without SYNCSRC  2
*/
#define IWSN_NODE_TYPE    SET_NODE_TYPE

#define SN_ROLE_NONE	  0x00
#define SN_ROLE_MANAGER   0x01
#define SN_ROLE_SYNCSRC   0x02
#define SN_ROLE_ROUTER    0x04
#define SN_ROLE_GATEWAY   0x08
#define SN_ROLE_PROXY     0x10
#define SN_ROLE_BRIDGE    0x20
#define SN_ROLE_SENSOR    0x40

#define SN_ROLE_ALL       (SN_ROLE_MANAGER | \
                           SN_ROLE_SYNCSRC | \
                           SN_ROLE_ROUTER  | \
                           SN_ROLE_GATEWAY | \
                           SN_ROLE_PROXY   | \
                           SN_ROLE_BRIDGE  | \
                           SN_ROLE_SENSOR)

#define SN_ABILITY_ALL    SN_ROLE_ALL

//=========================== typedef =========================================

typedef struct {
  uint8_t	myAbility;
  uint8_t	myRole;
  iwsn_addr_t   my16bID;
  iwsn_addr_t   myPANID;
  uint8_t   channel;
} idmanager_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void         idmanager_init();
uint8_t      idmanager_getMyAbility() ;
uint8_t      idmanager_getMyRole();
uint8_t 	 idmanager_getChannel();
void 		 idmanager_resetChannel();
void         idmanager_setMyRole(uint8_t newRole);
iwsn_addr_t* idmanager_getMyID(uint8_t type);
error_t      idmanager_setMyID(iwsn_addr_t* newID);
bool         idmanager_isSameAddress(iwsn_addr_t* address_1, iwsn_addr_t* address_2);
bool         idmanager_isMyAddress(iwsn_addr_t* addr);
bool         idmanager_isSyncSrc(uint8_t role);
bool         idmanager_isGateway(uint8_t role);
bool         idmanager_isProxy(uint8_t role);
bool         idmanager_isSensor(uint8_t role);
bool         idmanager_isRouter(uint8_t role);

#endif

