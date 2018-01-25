/**
\brief General IWSN "applications.c"

\author Hongchao Wang <hcwang@bjtu.edu.cn>, June 2015.
*/

//=========================== include =========================================
#include "applications.h"

#ifdef HAVE_BLINK
#include "blink.h"
#endif /* HAVE_BLINK */

#ifdef HAVE_ECHO
#include "echo.h"
#endif /* HAVE_ECHO */

#ifdef HAVE_IWSN
#include "iwsn.h"
#endif /* HAVE_IWSN */

#include "vibration.h"
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void applications_init(void)
{
#ifdef HAVE_BLINK
  //blink_init();
#endif /* HAVE_BLINK */
  
#ifdef HAVE_ECHO
  echo_init();
#endif /* HAVE_ECHO */

#ifdef HAVE_IWSN
  //iwsn_init();
#endif /* HAVE_IWSN */

  vibration_init();
}

void applications_start(void)
{  
#ifdef HAVE_BLINK
  //blink_start();
#endif /* HAVE_BLINK */
  
#ifdef HAVE_ECHO
  echo_start();
#endif /* HAVE_ECHO */

#ifdef HAVE_IWSN
  //iwsn_start();
#endif /* HAVE_IWSN */

  vibration_start();

}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
