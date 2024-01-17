#include "Task_Scheduler.h"
#include "stm32f1xx_hal.h"

sTaskTCB hSCH_tasks_G[hSCH_MAX_TASKS];


void hSCH_Init(void)
{
    uint8_t i;
    for(i = 0;i < hSCH_MAX_TASKS;i ++)
    {
        hSCH_tasks_G[i].pTask  = NULL;
        hSCH_tasks_G[i].Delay  = 0;
        hSCH_tasks_G[i].Period = 0;
        hSCH_tasks_G[i].RunMe  = 0;
        hSCH_tasks_G[i].Co_op  = 1;
        hSCH_tasks_G[i].Enable = 0;
    }
}


void SCH_Start(TIM_HandleTypeDef *htim)
{

	HAL_TIM_Base_Start_IT(htim);
}


void hSCH_Dispatch_Tasks(void)
{
   uint8_t Index;

    for (Index = 0; Index < hSCH_MAX_TASKS; Index++)
    {
        if ((hSCH_tasks_G[Index].Co_op) && (hSCH_tasks_G[Index].RunMe > 0))
        {
            if(hSCH_tasks_G[Index].pTask != NULL)
            {
                hSCH_tasks_G[Index].pTask();
            }
            hSCH_tasks_G[Index].RunMe -= 1;
        }

        if (hSCH_tasks_G[Index].Period == 0)
        {
            hSCH_tasks_G[Index].pTask = NULL;
        }
    }
}

sTaskTCB *SCH_Add_Task(TASK_SCH *pFunction, uint32_t DELAY, uint32_t PERIOD,uint8_t task_mode,uint8_t ENABLE)
{
    static uint8_t Index = 0;
    if (Index < hSCH_MAX_TASKS)
    {
        hSCH_tasks_G[Index].pTask  = pFunction;
        hSCH_tasks_G[Index].Delay  = DELAY;
        hSCH_tasks_G[Index].Period = PERIOD;
        hSCH_tasks_G[Index].RunMe  = 0;
        hSCH_tasks_G[Index].Co_op  = task_mode;
        hSCH_tasks_G[Index].Enable = ENABLE;
        Index++;
    }
    return &hSCH_tasks_G[Index-1];
}


void SCH_Delete_Task(sTaskTCB *task)
{
    task->pTask = NULL;
    task->Delay   = 0;
    task->Period  = 0;
    task->RunMe   = 0;
    task->Enable  = 0;
}


void SCH_Enable_Task(sTaskTCB *task)
{
    task->Enable  = 1;
    //task->Delay = task->Period;
}


void SCH_Disable_Task(sTaskTCB *task)
{
    task->Enable  = 0;
    task->Delay = 0;
}


void SCH_Delay_Task(sTaskTCB *task, uint32_t delaytime)
{
    task->Delay  += delaytime;
}


void SCH_Update(void)
{
	uint8_t Index_temp;

    for (Index_temp = 0; Index_temp < hSCH_MAX_TASKS; Index_temp++)
    {
        if(hSCH_tasks_G[Index_temp].Enable)
        {
            if(hSCH_tasks_G[Index_temp].Delay == 0)
            {
                if(hSCH_tasks_G[Index_temp].Co_op)
                {
                    hSCH_tasks_G[Index_temp].RunMe += 1;
                }
                else
                {
                    if( hSCH_tasks_G[Index_temp].pTask != NULL )
                    {
                        hSCH_tasks_G[Index_temp].pTask();
                    }
                }

                if(hSCH_tasks_G[Index_temp].Period)
                {
                    hSCH_tasks_G[Index_temp].Delay = (hSCH_tasks_G[Index_temp].Period - 1);
                }
            }
            else
            {
                hSCH_tasks_G[Index_temp].Delay -= 1;
            }
         }
      }
}


/************************ (C) COPYRIGHT HARRY007 *****END OF FILE**************/

