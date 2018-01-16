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

#ifdef HAVE_CONSOLE
#include "console.h"
#endif /* HAVE_CONSOLE */

#ifdef HAVE_TRX
#include "trx.h"
#endif /* HAVE_TRX */

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void applications_init(void)
{
#ifdef HAVE_BLINK
  blink_init();
#endif /* HAVE_BLINK */
  
#ifdef HAVE_ECHO
  echo_init();
#endif /* HAVE_ECHO */
  
#ifdef HAVE_CONSOLE
  console_init();
#endif /* HAVE_CONSOLE */
  
#ifdef HAVE_TRX
  trx_init();
#endif /* HAVE_TRX */
}

void applications_start(void)
{  
#ifdef HAVE_BLINK
  blink_start();
#endif /* HAVE_BLINK */
  
#ifdef HAVE_ECHO
  echo_start();
#endif /* HAVE_ECHO */
  
#ifdef HAVE_CONSOLE
  console_start();
#endif /* HAVE_CONSOLE */
  
#ifdef HAVE_TRX
  trx_start();
#endif /* HAVE_TRX */
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================