
#include "SW_control_task.h"
#include "chassis_power_control.h"
steering_wheel_t steering_wheel;

int16_t probe;
void SW_control_task(void)
{
    if(steering_wheel.parameter.enable)
	{
        
        Steering_Wheel_CommandUpdate(&steering_wheel);
        Steering_Wheel_StatusUpdate(&steering_wheel);
		Steering_Wheel_CommandTransmit(&steering_wheel);
		
		
		//É¾³ý²âÊÔ
		//probe = steering_wheel.directive_part.motor.M3508_kit.parameter.bus->motor[1].feedback.LSB_rotor_rpm;


      }
        
    
    

}

void SW_subscribe_task(void)
{
    steering_communication_SubscribeList_Scheduler(&steering_wheel);
}

void SW_control_task_init(void)
{

    Steering_Wheel_HandleInit(&steering_wheel);
	steering_communication_init();
}
