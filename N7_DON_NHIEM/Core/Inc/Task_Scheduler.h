
#ifndef __TASK_SCHEDULER_H
#define __TASK_SCHEDULER_H

#include "stm32f4xx_hal.h"

#define hSCH_MAX_TASKS  4
typedef void TASK_SCH(void);

typedef struct
{
	uint8_t  Enable;
	uint8_t  RunMe;
	uint8_t  Co_op;
	uint32_t Delay;
	uint32_t Period;
    TASK_SCH *pTask;
}sTaskTCB;

extern sTaskTCB hSCH_tasks_G[hSCH_MAX_TASKS];


void hSCH_Init(void);


void SCH_Start(TIM_HandleTypeDef *htim);


void hSCH_Dispatch_Tasks(void);

sTaskTCB *SCH_Add_Task(TASK_SCH *pFunction, uint32_t DELAY, uint32_t PERIOD,uint8_t task_mode,uint8_t ENABLE);

void SCH_Delete_Task(sTaskTCB *task);

void SCH_Enable_Task(sTaskTCB *task);

void SCH_Disable_Task(sTaskTCB *task);

void SCH_Delay_Task(sTaskTCB *task, uint32_t delaytime);

void SCH_Update(void);


#endif

