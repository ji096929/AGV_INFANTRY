
// Comment TIM3_IRQHandler to enable this scheduler


// Include
#include "main.h"
#include "SW_control_task.h"
int32_t ms_count;
int32_t s_count;
uint32_t total_count;
// Function Call
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
	SW_control_task();
	SW_subscribe_task();
	total_count=s_count*1000+ms_count;
	ms_count++;
	if(ms_count==1000)
	{
		ms_count=0;
		s_count++;
	}
}
}


// Scheduler

void TASK_SCHEDULER(void)
{
//	uint32_t tick = HAL_GetTick();
	SW_control_task();
	//SW_subscribe_task();
}


