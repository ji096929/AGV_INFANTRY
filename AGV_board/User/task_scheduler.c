
// Comment TIM3_IRQHandler to enable this scheduler

//#define motor_power_test

// Include
#include "main.h"
#include "math.h"
#include "SW_control_task.h"
#include "chassis_power_control.h"
#include "buzzer.h"
int32_t ms_count;
int32_t s_count;
uint32_t total_count;
TEST_POWER_T test_power;
// Function Call
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		#ifndef motor_power_test
		SW_control_task();
		
		//SW_subscribe_task();
		#endif
		#ifdef motor_power_test
		if(test_power.test_mode==1)
		{
		steering_wheel.motion_part.motor.M3508_kit.command.torque=test_power.target_current;
		}
		else if(test_power.test_mode ==2)
		{
		steering_wheel.motion_part.motor.M3508_kit.command.torque=test_power.target_current*(float)sin((double)(ms_count*test_power.temp_a)+test_power.temp_b);
		}
		Steering_Wheel_StatusUpdate(&steering_wheel);
		Steering_Wheel_CommandTransmit(&steering_wheel);
		#endif
	total_count=s_count*1000+ms_count;
	ms_count++;
	if(ms_count==1000)
	{
		ms_count=0;
		s_count++;
	}
	
	
}
static uint32_t cnt=0;	
if(htim->Instance==TIM2)
{
	cnt++;
		buzzer_taskScheduler(&buzzer);
	        //buzzer_setTask(&buzzer, BUZZER_CALIBRATING_PRIORITY);
        if(cnt>100)
	  {
		  Alive_Tect();
		  cnt=0;
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


