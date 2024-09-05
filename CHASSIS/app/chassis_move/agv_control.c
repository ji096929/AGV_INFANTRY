#include "agv_control.h"
#include "chassis.h"
#include "referee.h"

chassis_power_control_t chassis_power_control;
PID_TypeDef buffer_pid;
PID_TypeDef supercap_pid;
void Set_AGV_Velocity_Vector_Data_Update(uint8_t tx_data[], int16_t angle, int16_t speed, float power_limition)
{

    memcpy(&tx_data[0], &angle, 2);
    memcpy(&tx_data[2], &speed, 2);
    memcpy(&tx_data[4], &power_limition, 4);
}
void AGV_connoection(int ms_cnt)
{

    if (ms_cnt % 12 == 0)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            if (chassis.A_motor.status == Status_Off)
            {
                chassis.A_motor.status = Status_On;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.A_motor.ID, 0, 0x01), AGV_A_Tx_Data, 8);
            }

            Set_AGV_Velocity_Vector_Data_Update(AGV_A_Tx_Data, chassis.A_motor.target_angle, chassis.A_motor.target_speed.output, chassis_power_control.scaled_power_32[0]);
        }
        else
        {

            if (chassis.A_motor.status == Status_On)
            {
                chassis.A_motor.status = Status_Off;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.A_motor.ID, 0, 0x00), AGV_A_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_A_Tx_Data, chassis.A_motor.target_angle, 0, chassis_power_control.scaled_power_32[0]);
        }
        CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.A_motor.ID, 0, 0x03), AGV_A_Tx_Data, 8);
    }
    if (ms_cnt % 12 == 3)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            if (chassis.B_motor.status == Status_Off)
            {
                chassis.B_motor.status = Status_On;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.B_motor.ID, 0, 0x01), AGV_B_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_B_Tx_Data, chassis.B_motor.target_angle, chassis.B_motor.target_speed.output, chassis_power_control.scaled_power_32[1]);
        }
        else
        {
            if (chassis.B_motor.status == Status_On)
            {
                chassis.B_motor.status = Status_Off;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.B_motor.ID, 0, 0x00), AGV_A_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_B_Tx_Data, chassis.B_motor.target_angle, 0, chassis_power_control.scaled_power_32[1]);
        }
        CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.B_motor.ID, 0, 0x03), AGV_B_Tx_Data, 8);
    }
    if (ms_cnt % 12 == 6)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            if (chassis.C_motor.status == Status_Off)
            {
                chassis.C_motor.status = Status_On;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.C_motor.ID, 0, 0x01), AGV_B_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_C_Tx_Data, chassis.C_motor.target_angle, chassis.C_motor.target_speed.output, chassis_power_control.scaled_power_32[2]);
        }
        else
        {
            if (chassis.C_motor.status == Status_On)
            {
                chassis.C_motor.status = Status_Off;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.C_motor.ID, 0, 0x00), AGV_A_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_C_Tx_Data, chassis.C_motor.target_angle, 0, chassis_power_control.scaled_power_32[2]);
        }
        CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.C_motor.ID, 0, 0x03), AGV_C_Tx_Data, 8);
    }
    if (ms_cnt % 12 == 9)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            if (chassis.D_motor.status == Status_Off)
            {
                chassis.D_motor.status = Status_On;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.D_motor.ID, 0, 0x01), AGV_B_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_D_Tx_Data, chassis.D_motor.target_angle, chassis.D_motor.target_speed.output, chassis_power_control.scaled_power_32[3]);
        }
        else
        {
            if (chassis.D_motor.status == Status_On)
            {
                chassis.D_motor.status = Status_Off;
                CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.D_motor.ID, 0, 0x00), AGV_A_Tx_Data, 8);
            }
            Set_AGV_Velocity_Vector_Data_Update(AGV_D_Tx_Data, chassis.D_motor.target_angle, 0, chassis_power_control.scaled_power_32[3]);
        }
        CAN_Send_EXT_Data(&hcan1, EXT_ID_Set(chassis.D_motor.ID, 0, 0x03), AGV_D_Tx_Data, 8);
    }
}
float test_sum;
void calculate_true_power(void)
{
    float expect_supercap_per;
    PID_Calculate(&buffer_pid, JudgeReceive.remainEnergy, 30);
    float sum = 0;
    if (JudgeReceive.MaxPower == 65535)
        JudgeReceive.MaxPower = 5000;

    if (JudgeReceive.MaxPower == 0)
        chassis_power_control.power_limit_max = 80;
    else
        chassis_power_control.power_limit_max = JudgeReceive.MaxPower - buffer_pid.Output;

    // chassis_power_control.power_limit_max=chassis_power_control.power_limit_max+(chassis.supercap.supercap_per*100-30)*2;

    //    if (chassis.supercap.supercap_per > 5)
    //    {
    if (chassis.supercap.state == 0)
    {
        expect_supercap_per = 0.6;
        // chassis_power_control.power_limit_max = chassis_power_control.power_limit_max + 5; // slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
    }
    else
    {
        expect_supercap_per = -0.5;
        // chassis_power_control.power_limit_max = chassis_power_control.power_limit_max + 80;
    }
    PID_Calculate(&supercap_pid, chassis.supercap.supercap_per, expect_supercap_per);
    chassis_power_control.power_limit_max -= supercap_pid.Output;

    //chassis_power_control.power_limit_max += (chassis.supercap.supercap_per - expect_supercap_per) * 400;

    if (chassis.supercap.supercap_voltage < 13.5)
    {
        chassis_power_control.power_limit_max = JudgeReceive.MaxPower-5;
    }
    //    }
    //    else
    //    {
    //        chassis_power_control.power_limit_max = chassis_power_control.power_limit_max;
    //    }

    if (chassis_power_control.all_mscb_ready_flag & 0xf)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            sum += chassis_power_control.expect_power_32[i];
        }
        test_sum = sum;

        chassis_power_control.scaled_power_coefficient_32 = (chassis_power_control.power_limit_max) / sum;
        if (chassis_power_control.scaled_power_coefficient_32 <= 1)
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
        chassis_power_control.all_mscb_ready_flag = 0;
    }
}

void Chassis_Power_Control_Init(void)
{
    for (int i = 0; i < 4; i++)
    {
        chassis_power_control.scaled_power_32[i] = 60.0f;
    }
}