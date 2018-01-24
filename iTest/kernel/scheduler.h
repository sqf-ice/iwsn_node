/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: scheduler.h
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#ifdef __cplusplus
extern "C" {
#endif

//=========================== include =========================================
#include "platform.h"

//=========================== define ==========================================
#define TASK_LIST_DEPTH         10

//=========================== typedef =========================================
typedef enum 
{
  TASK_PRIO_NONE = 0x00,
  TASK_PRIO_1 = 0x01,
  TASKPRIO_RESNOTIF_RX =0x01,
  TASKPRIO_RESNOTIF_TXDONE =0x02,
  TASK_PRIO_2 = 0x02,
  TASK_PRIO_3 = 0x03,
  TASKPRIO_RES = 0x03,
  TASK_PRIO_4 = 0x04,
  TASK_PRIO_5 = 0x05,
  TASK_PRIO_6 = 0x06,
  TASK_PRIO_7 = 0x07,
  TASK_PRIO_8 = 0x08,
  TASK_PRIO_9 = 0x09,
  TASK_PRIO_MAX
} task_prio_t;   //优先级的值越小越能优先执行

typedef void (*task_cbt)(void);

typedef struct task_llist_t
{
  task_cbt      cb;
  task_prio_t   prio;
  void*         next;
} taskList_item_t;  //单链表

typedef struct 
{
  taskList_item_t       taskBuf[TASK_LIST_DEPTH];
  taskList_item_t*      task_list;
  uint8_t               numTasksCur;
  uint8_t               numTasksMax;
} scheduler_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================
void scheduler_init(void);
void scheduler_start(void);
void scheduler_push_task(task_cbt cb, task_prio_t prio);

//=========================== macros ==========================================

#ifdef __cplusplus
}
#endif

#endif /* _SCHEDULER_H_ */
