/**
\brief General IWSN Cross-layer 'idmanager' definitions.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "string.h"
#include "idmanager.h"
#include "info.h"
#include "board.h"

#ifdef HAVE_DEBUG
#include "debug.h"
#endif

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

idmanager_vars_t idmanager_vars;

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================

void idmanager_init()
{
  // Initialize variables
  memset(&idmanager_vars, 0, sizeof(idmanager_vars_t));
  
  idmanager_vars.myAbility            = SN_ABILITY_ALL;
  idmanager_vars.myRole               = SN_ROLE_NONE;
  
  idmanager_vars.my16bID.type         = ADDR_16B;
  info_get16BID(idmanager_vars.my16bID.addr_16b);
  
  idmanager_vars.myPANID.type         = ADDR_PANID;
  info_getPANID(idmanager_vars.myPANID.panid);
  
  if (IWSN_NODE_TYPE == 0)
  {
    idmanager_setMyRole(SN_ROLE_ROUTER  |
                        SN_ROLE_PROXY   |
                        SN_ROLE_SENSOR);
  }
  else if (IWSN_NODE_TYPE == 1)
  {
    idmanager_setMyRole(SN_ROLE_GATEWAY | 
                        SN_ROLE_SYNCSRC |
                        SN_ROLE_ROUTER  |
                        SN_ROLE_PROXY   | 
                        SN_ROLE_SENSOR);
  }
  else if (IWSN_NODE_TYPE == 2)
  {
    idmanager_setMyRole(SN_ROLE_GATEWAY | 
                        SN_ROLE_ROUTER  |
                        SN_ROLE_PROXY   | 
                        SN_ROLE_SENSOR);
  }
}

uint8_t idmanager_getMyAbility() 
{
  uint8_t res;
  
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  res = idmanager_vars.myAbility;
  ENABLE_INTERRUPTS();
  
  return res;
}

uint8_t idmanager_getMyRole() 
{
  uint8_t res;
  
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  res = idmanager_vars.myRole;
  ENABLE_INTERRUPTS();
  
  return res;
}

void idmanager_setMyRole(uint8_t newRole) 
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  idmanager_vars.myRole = newRole;
  
  ENABLE_INTERRUPTS();
}

iwsn_addr_t* idmanager_getMyID(uint8_t type)
{
  iwsn_addr_t* res;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  switch (type)
  {
    case ADDR_16B:
      res = &idmanager_vars.my16bID;
      break;
    case ADDR_PANID:
      res = &idmanager_vars.myPANID;
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_IDMANAGER, ERR_WRONG_ADDR_TYPE,
                       (errorparameter_t)type,
                       (errorparameter_t)0);
#endif
      res = NULL;
      break;
  }
  ENABLE_INTERRUPTS();
  
  return res;
}

error_t idmanager_setMyID(iwsn_addr_t* newID) 
{
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  switch (newID->type) 
  {
    case ADDR_16B:
      memcpy(&idmanager_vars.my16bID, newID, sizeof(iwsn_addr_t));
      break;
    case ADDR_PANID:
      memcpy(&idmanager_vars.myPANID, newID, sizeof(iwsn_addr_t));
      break;
    default:
#ifdef HAVE_DEBUG
      debug_printError(COMPONENT_IDMANAGER, ERR_WRONG_ADDR_TYPE,
                       (errorparameter_t)newID->type,
                       (errorparameter_t)1);
#endif
      ENABLE_INTERRUPTS();
      return E_FAIL;
  }
  ENABLE_INTERRUPTS();
  
  return E_SUCCESS;
}

bool idmanager_isSameAddress(iwsn_addr_t* address_1, iwsn_addr_t* address_2)
{
  uint8_t address_length;
  
  if (address_1->type != address_2->type)
  {
    return false;
  }
  
  switch (address_1->type)
  {
    case ADDR_16B:
    case ADDR_PANID:
      address_length = 2;
      break;
    default:
#if HAVE_DEBUG
      debug_printError(COMPONENT_IDMANAGER, ERR_WRONG_ADDR_TYPE,
                       (errorparameter_t)address_1->type,
                       (errorparameter_t)5);
#endif
      break;
  }
  
  if (memcmp((void *)address_1->addr_16b, (void*)address_2->addr_16b, address_length) == 0)
  {
    return true;
  }

  return false;
}

bool idmanager_isMyAddress(iwsn_addr_t* addr)
{
  bool res;  
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  
  switch (addr->type)
  {
    case ADDR_16B:
      res = idmanager_isSameAddress(addr, &idmanager_vars.my16bID);
      ENABLE_INTERRUPTS();
      return res;
    case ADDR_PANID:
      res= idmanager_isSameAddress(addr, &idmanager_vars.myPANID);
      ENABLE_INTERRUPTS();
      return res;
    default:
      ENABLE_INTERRUPTS();
      return false;
  }
}

bool idmanager_isSyncSrc(uint8_t role) 
{  
  return (role & SN_ROLE_SYNCSRC) ? true : false;
}

bool idmanager_isGateway(uint8_t role) 
{  
  return (role & SN_ROLE_GATEWAY) ? true : false;
}

bool idmanager_isProxy(uint8_t role) 
{  
  return (role & SN_ROLE_PROXY) ? true : false;
}

bool idmanager_isSensor(uint8_t role) 
{  
  return (role & SN_ROLE_SENSOR) ? true : false;
}

bool idmanager_isRouter(uint8_t role) 
{  
  return (role & SN_ROLE_ROUTER) ? true : false;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
