/*
 * Copyright: Beijing Mesh Technology Co. Ltd, 2016-2020.
 * Filename: scheduler.c
 * Author: Hongchao Wang <hcwang@bjtu.edu.cn>
 * Date: Jun 12th, 2017
 * Function: the source/header of the project
 */

//=========================== include =========================================
#include "scheduler.h"
#include "board.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
scheduler_vars_t scheduler_vars;

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void scheduler_init(void) 
{
  /* Initialize module variables. */
  memset(&scheduler_vars, 0, sizeof(scheduler_vars_t));

  /* Enable the scheduler's interrupt so SW can wake up the scheduler. */
  //SCHEDULER_ENABLE_INTERRUPT();
}

void scheduler_start(void) 
{
  taskList_item_t* pThisTask;

  while (1) 
  {
    while (scheduler_vars.task_list != NULL) 
    {
      /* There is still at least one task in the linked-list of tasks. */

      /* The task to execute is the one at the head of the queue. */
      pThisTask = scheduler_vars.task_list;

      /* Shift the queue by one task. */
      scheduler_vars.task_list = pThisTask->next;

      /* Execute the current task. */
      pThisTask->cb();

      /* Free up this task container. */
      pThisTask->cb = NULL;
      pThisTask->prio = TASK_PRIO_NONE;
      pThisTask->next = NULL;

      /* Reduce the task number by one. */
      scheduler_vars.numTasksCur--;
    }
    //board_sleep();
  }
}

void scheduler_push_task(task_cbt cb, task_prio_t prio) 
{
  taskList_item_t* taskContainer;
  taskList_item_t** taskListWalker;   //指向单链表内部的next字段
  //INTERRUPT_DECLARATION();

  DISABLE_INTERRUPTS();

  /* Find an empty task container. */
  taskContainer = &scheduler_vars.taskBuf[0];
  while (taskContainer->cb != NULL 
         && taskContainer <= &scheduler_vars.taskBuf[TASK_LIST_DEPTH - 1]) 
  {
    taskContainer++;
  }
  if (taskContainer > &scheduler_vars.taskBuf[TASK_LIST_DEPTH - 1]) 
  {
    /* The task list has overflown. */
    return;
  }

  /* Fill that task container with this task. */
  taskContainer->cb = cb;
  taskContainer->prio = prio;

  /* Find position in queue. */
  taskListWalker = &scheduler_vars.task_list;
  while (*taskListWalker != NULL && (*taskListWalker)->prio < taskContainer->prio) 
  {
    taskListWalker = (taskList_item_t**) &((*taskListWalker)->next);
  }

  /* Insert the task at that position. */
  taskContainer->next = *taskListWalker;
  *taskListWalker = taskContainer;

  /* Maintain scheduler stats. */
  scheduler_vars.numTasksCur++;
  if (scheduler_vars.numTasksCur > scheduler_vars.numTasksMax) 
  {
    scheduler_vars.numTasksMax = scheduler_vars.numTasksCur;
  }

  ENABLE_INTERRUPTS();
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
