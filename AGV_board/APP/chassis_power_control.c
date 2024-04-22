#include "chassis_power_control.h"
#include "SW_control_task.h"
chassis_power_control_t chassis_power_control;
int sign( int x )
{ 
     if(x>0)
     return 1;
    else if(x==0)
    return 0;
    else
    return -1; 
}

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

    float b, c, temp;
    b = steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
    c = K2 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm - scaled_power + CONSTANT;
    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        temp = (-b + Sqrt(b * b - 4 * K1 * c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque <= temp)
        {
            steering_wheel.motion_part.motor.M3508_kit.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (temp > 16000)
            {
                steering_wheel.motion_part.motor.M3508_kit.command.torque = 16000;
            }
            else
                steering_wheel.motion_part.motor.M3508_kit.command.torque = (int)temp;
        }
    }
    else
    {
        temp = (-b - Sqrt(b * b - 4 * K1 * c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque >= temp)
        {
            steering_wheel.motion_part.motor.M3508_kit.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (temp < -16000)
            {
                steering_wheel.motion_part.motor.M3508_kit.command.torque = -16000;
            }
            else
                steering_wheel.motion_part.motor.M3508_kit.command.torque = (int)temp;
        }
    }
}

float expert_power_calculate(void)
{
//    chassis_power_control.expect_motion_part_power_32 =
//        steering_wheel.motion_part.motor.command.torque * TOQUE_COEFFICIENT * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm +
//        K1 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm +
//        K2 * steering_wheel.motion_part.motor.command.torque * steering_wheel.motion_part.motor.command.torque + CONSTANT;

	    chassis_power_control.expect_motion_part_power_32 =
        steering_wheel.motion_part.motor.command.torque * TOQUE_COEFFICIENT * __fabs(steering_wheel.motion_part.command.protocol_speed)*14*sign(steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm) +
        K1 * steering_wheel.motion_part.command.protocol_speed*14 * steering_wheel.motion_part.command.protocol_speed *14+
        K2 * steering_wheel.motion_part.motor.command.torque * steering_wheel.motion_part.motor.command.torque + CONSTANT;

	
    // chassis_power_control.expect_directive_part_power_32 =
    //     steering_wheel.directive_part.motor.command.torque * TOQUE_COEFFICIENT * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm +
    //     K2 * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm +
    //     K1 * steering_wheel.directive_part.motor.command.torque * steering_wheel.directive_part.motor.command.torque + CONSTANT;

    chassis_power_control.expect_total_power_32 = chassis_power_control.expect_motion_part_power_32 ;
}

float m_b, m_c, m_temp, d_b, d_c, d_temp;
float scaled_power_calculate(void)
{

    chassis_power_control.scaled_power_coefficient_32 = chassis_power_control.power_limit_max / chassis_power_control.expect_total_power_32;
    chassis_power_control.scaled_directive_part_power_32 = chassis_power_control.expect_directive_part_power_32 * chassis_power_control.scaled_power_coefficient_32;
    chassis_power_control.scaled_motion_part_power_32 = chassis_power_control.expect_motion_part_power_32 * chassis_power_control.scaled_power_coefficient_32;
    m_b = steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
    m_c = K2 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm;
    m_c = m_c - chassis_power_control.scaled_motion_part_power_32 + CONSTANT;
    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        m_temp = (-m_b + Sqrt(m_b * m_b - 4 * K1 * m_c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque <= m_temp)
        {
            steering_wheel.motion_part.motor.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (m_temp > 16000)
            {
                steering_wheel.motion_part.motor.command.torque = 16000;
            }
            else
                steering_wheel.motion_part.motor.command.torque = (int)m_temp;
        }
    }
    else
    {
        m_temp = (-m_b - Sqrt(m_b * m_b - 4 * K1 * m_c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque >= m_temp)
        {
            steering_wheel.motion_part.motor.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (m_temp < -16000)
            {
                steering_wheel.motion_part.motor.command.torque = -16000;
            }
            else
                steering_wheel.motion_part.motor.command.torque = (int)m_temp;
        }
    }
    d_b = steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
    d_c = K2 * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm - chassis_power_control.scaled_directive_part_power_32 + CONSTANT;
    if (steering_wheel.directive_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        d_temp = (-d_b + Sqrt(d_b * d_b - 4 * K1 * d_c)) / (2 * K1);

        if (steering_wheel.directive_part.motor.command.torque <= d_temp)
        {
            steering_wheel.directive_part.motor.command.torque = steering_wheel.directive_part.motor.command.torque;
        }
        else
        {
            if (d_temp > 16000)
            {
                steering_wheel.directive_part.motor.command.torque = 16000;
            }
            else
                steering_wheel.directive_part.motor.command.torque = (int)d_temp;
        }
    }
    else
    {
        d_temp = (-d_b - Sqrt(d_b * d_b - 4 * K1 * d_c)) / (2 * K1);

        if (steering_wheel.directive_part.motor.command.torque >= d_temp)
        {
            steering_wheel.directive_part.motor.command.torque = steering_wheel.directive_part.motor.command.torque;
        }
        else
        {
            if (d_temp < -16000)
            {
                steering_wheel.directive_part.motor.command.torque = -16000;
            }
            else
                steering_wheel.directive_part.motor.command.torque = (int)d_temp;
        }
    }
}