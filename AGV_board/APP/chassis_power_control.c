#include "chassis_power_control.h"
#include "SW_control_task.h"
chassis_power_control_t chassis_power_control;

long double view_b;
long double view_c;
long double view_temp;
long double temp1,temp2,temp3,temp4;
long double b;
long double c;
		
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
		
long double calculate_torque_current_according_to_scaled_power(long double scaled_power)
{



	b = steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
     c = K2 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm - scaled_power + CONSTANT;
    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
         temp1 = (-b + Sqrt(b * b - 4 * K1 * c)) / (2 * K1);
			view_temp	=	temp3;
        if (temp3 > 16000)
        {
            steering_wheel.motion_part.motor.command.torque = 16000;
        }
        else
            steering_wheel.motion_part.motor.command.torque = (int)temp3;
    }
    else
    {
//         temp = (-b - sqrt(b * b - 4 * K1 * c)) / (2 * K1);
					temp1	=	Sqrt((b * b) - (4.0f * K1 * c));
					temp2	=	-temp1	-	b;
				temp3	=	temp2/2.0f/K1;
			 view_temp	=	temp3;
        if (temp3 < -16000)
        {
            steering_wheel.motion_part.motor.command.torque = -16000;
        }
        else
            steering_wheel.motion_part.motor.command.torque = (int)temp3;
    }
}