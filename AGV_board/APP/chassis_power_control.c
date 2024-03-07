#include "chassis_power_control.h"
#include "SW_control_task.h"
chassis_power_control_t chassis_power_control;

float calculate_torque_current_according_to_scaled_power(float scaled_power)
{
    float b = steering_wheel.motion_part.motor.M3508_kit.status.output_speed_rpm * TOQUE_COEFFICIENT;
    float c = K2 * steering_wheel.motion_part.motor.M3508_kit.status.output_speed_rpm * steering_wheel.motion_part.motor.M3508_kit.status.output_speed_rpm - scaled_power + CONSTANT;

    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        float temp = (-b + sqrt(b * b - 4 * K1 * c)) / (2 * K1);
        if (temp > 16000)
        {
            steering_wheel.motion_part.motor.command.torque = 16000;
        }
        else
            steering_wheel.motion_part.motor.command.torque = temp;
    }
    else
    {
        float temp = (-b - sqrt(b * b - 4 * K1 * c)) / (2 * K1);
        if (temp < -16000)
        {
            steering_wheel.motion_part.motor.command.torque = -16000;
        }
        else
            steering_wheel.motion_part.motor.command.torque = temp;
    }
}