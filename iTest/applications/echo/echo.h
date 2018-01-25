/**
\brief General IWSN "echo.c"

\author Hongchao Wang <hcwang@bjtu.edu.cn>, Yipeng Cun <cunyipeng@bjtu.edu.cn>, Jan 2018.
*/

#ifndef __ECHO_H
#define __ECHO_H

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"
#include "scheduler.h"
#include "timers.h"
#include "uart.h"

//=========================== define ==========================================
#define ECHO_BUF_MAX          256
  
//=========================== typedef =========================================
typedef struct
{
  uint8_t       buf[ECHO_BUF_MAX];
  uint8_t       rxIdx;
  uint8_t       txIdx;
  uint16_t       txSize;
} echo_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void echo_init(void);
void echo_start(void);


//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* __ECHO_H */
