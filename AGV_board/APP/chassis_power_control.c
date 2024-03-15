#include "chassis_power_control.h"
#include "SW_control_task.h"
chassis_power_control_t chassis_power_control;


		
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}		
		
float calculate_torque_current_according_to_scaled_power(float scaled_power)
{

			float	b,c,temp;	
	b = steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
     c = K2 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm - scaled_power + CONSTANT;
    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
         temp = (-b + Sqrt(b * b - 4 * K1 * c)) / (2 * K1);
			
					if(steering_wheel.motion_part.motor.command.torque <=	temp)
					{
						steering_wheel.motion_part.motor.command.torque=steering_wheel.motion_part.motor.command.torque;
					}
					else 
					{
				if (temp > 16000)
        {
            steering_wheel.motion_part.motor.command.torque = 16000;
        }
        else
            steering_wheel.motion_part.motor.command.torque = (int)temp;
					}
			
				
    }
    else
    {
         temp = (-b - Sqrt(b * b - 4 * K1 * c)) / (2 * K1);

      
					if(steering_wheel.motion_part.motor.command.torque >=	temp)
					{
						steering_wheel.motion_part.motor.command.torque=steering_wheel.motion_part.motor.command.torque;
					}
					else   
					{
				if (temp < -16000)
        {
            steering_wheel.motion_part.motor.command.torque = -16000;
        }
        else
            steering_wheel.motion_part.motor.command.torque = (int)temp;
			}
    }
}