/**
\brief General IWSN definition

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

//=========================== include =========================================

#include "iwsn.h"
#include "platform.h"
//#include "serial.h"
#include "idmanager.h"
#include "queue.h"
#include "random.h"
#include "timers.h"
#include "ieee802154e.h"
#include "rtimer.h"
#include "radio.h"
#include "neighbors.h"
#include "tsch.h"
#include "res.h"
#include "network.h"
#include "route.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================

void iwsn_init()
{
  // ====== drivers 
#ifdef HAVE_SERIAL
  serial_init();
#endif
  
  // ====== stack
  // Cross-layer
  idmanager_init();
  queue_init();
  random_init();
  timers_init();
  
  // 02b-MAChigh
  neighbors_init();
  tsch_init();
  res_init();
  
  // 02a-MAClow
  ieee802154e_init();
  
  // 03-Network
  net_init();
  
  // 07-Application
}

void iwsn_start()
{
  // Start the radiotimer
  rtimer_start(TsSlotDuration);
  
  // Switch radio on
  radio_rfOn();
}

/**
/brief Difference between some older ASN and the newer ASN.

\param oldASN [in] some older ASN to compare to the newer
       newASN [in] the newer ASN

\returns The ASN difference, or 0xffff if more than 65535 different
*/
PORT_TIMER_WIDTH iwsn_asnDiff(asn_t* oldASN, asn_t* newASN)
{
  PORT_TIMER_WIDTH diff;
  INTERRUPT_DECLARATION();
  DISABLE_INTERRUPTS();
  if (newASN->byte4 != oldASN->byte4)
  {
    ENABLE_INTERRUPTS();
    return (PORT_TIMER_WIDTH)0xFFFFFFFF;
  }
  
  diff = 0;
  if (newASN->bytes2and3 == oldASN->bytes2and3)
  {
    ENABLE_INTERRUPTS();
    return newASN->bytes0and1 - oldASN->bytes0and1;
  }
  else if (newASN->bytes2and3 - oldASN->bytes2and3 == 1)
  {
    diff  = newASN->bytes0and1;
    diff += 0xFFFF - oldASN->bytes0and1;
    diff += 1;
  }
  else
  {
    diff  = (PORT_TIMER_WIDTH)0xFFFFFFFF;
  }
  ENABLE_INTERRUPTS();
  return diff;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
