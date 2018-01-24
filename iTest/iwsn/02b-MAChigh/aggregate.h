/**
\brief General IWSN 02bMAClow-high 'aggregate' declarations.

\author Dong Yang <dyang@bjtu.edu.cn>, August 2012.
*/

#ifndef __AGGREGATE_H
#define __AGGREGATE_H

//=========================== include =========================================

#include "timers.h"
#include "queue.h"

//=========================== define ==========================================

#define MIN_LENGTH_AGGREGATE   16
#define MAX_LENGTH_AGGREGATE   99

enum aggregate_type_enums {
  AGGREGATE_CAN_SAMPLE_COLLECT   = 5,
  AGGREGATE_SHT_SAMPLE_COLLECT   = 1,
  AGGREGATE_UART_SAMPLE_COLLECT	 = 2,
};

//=========================== typedef =========================================

typedef struct {
  uint16_t        periodMaintenance;
  timer_id_t      timerId;
} aggregate_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void    aggregate_init();
error_t aggregate_send(QueueEntry_t *msg);
void    aggregate_send_internal(QueueEntry_t *msg);
void    aggregate_appendData(QueueEntry_t *msg, uint8_t *dataToAppend, uint8_t length, uint8_t numOfBlock);
uint8_t aggregate_getLeftLength(QueueEntry_t *msg);

#endif
