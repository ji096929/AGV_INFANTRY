#include "chassis_power_control.h"

 chassis_power_control_t chassis_power_control;

void calculate_true_power(void)
{
    float sum = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        sum+=chassis_power_control.expect_power_32[i];
    }
    chassis_power_control.scaled_power_coefficient_32 = (chassis_power_control.power_limit_max) / sum;
    if (chassis_power_control.scaled_power_coefficient_32<=1)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_power_control.scaled_power_32[i] = chassis_power_control.scaled_power_coefficient_32 * chassis_power_control.expect_power_32[i];
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_power_control.scaled_power_32[i] = chassis_power_control.expect_power_32[i];
        }

    }
       
}

void Chassis_Power_Control_Init(void)
{
    
}


