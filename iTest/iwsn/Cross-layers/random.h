/**
\brief General IWSN Cross-layer 'random' specifications.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __RANDOM_H
#define __RANDOM_H

//=========================== include =========================================

#include "stdint.h"

//=========================== define ==========================================

//=========================== typedef =========================================

typedef struct {
  uint16_t shift_reg;  // Galois shift register used to obtain a pseudo-random number
} random_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void     random_init();
uint16_t random_get16b();

#endif
