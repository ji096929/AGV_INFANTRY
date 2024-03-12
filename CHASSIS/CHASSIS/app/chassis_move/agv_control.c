#include "agv_control.h"
#include "chassis.h"
 chassis_power_control_t chassis_power_control;

void Set_AGV_Velocity_Vector_Data_Update(uint8_t tx_data[],int16_t	angle,int16_t speed,float power_limition)
{
	
		memcpy(&tx_data[0],&angle,2);
		memcpy(&tx_data[2],&speed,2);
		memcpy(&tx_data[4],&power_limition,4);
		
	
	
}
void AGV_connoection(int ms_cnt)
{
	 if(chassis_power_control.all_mscb_ready_flag&0x01	&& ms_cnt%5==0)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                Set_AGV_Velocity_Vector_Data_Update(AGV_A_Tx_Data, chassis.A_motor.target_angle, chassis.A_motor.target_speed.output, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY);       
            }
            else
            {
							  Set_AGV_Velocity_Vector_Data_Update(AGV_A_Tx_Data, chassis.A_motor.target_angle, chassis.A_motor.target_speed.output, chassis_power_control.scaled_power_32[0]);   
            }
        }
        else
        {
								Set_AGV_Velocity_Vector_Data_Update(AGV_A_Tx_Data, chassis.A_motor.target_angle, 0, chassis_power_control.scaled_power_32[0]);   
        }
				CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.A_motor.ID,0,0x03),AGV_A_Tx_Data,8);
        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x0E;
    }
		if(chassis_power_control.all_mscb_ready_flag&0x02	&& ms_cnt%5==1)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                Set_AGV_Velocity_Vector_Data_Update(AGV_B_Tx_Data, chassis.B_motor.target_angle, chassis.B_motor.target_speed.output, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY);       
            }
            else
            {
							  Set_AGV_Velocity_Vector_Data_Update(AGV_B_Tx_Data, chassis.B_motor.target_angle, chassis.B_motor.target_speed.output, chassis_power_control.scaled_power_32[1]);   
            }
        }
        else
        {
								Set_AGV_Velocity_Vector_Data_Update(AGV_B_Tx_Data, chassis.B_motor.target_angle, 0, chassis_power_control.scaled_power_32[1]);   
        }
				CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.B_motor.ID,0,0x03),AGV_B_Tx_Data,8);
        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x0D;
    }
		if(chassis_power_control.all_mscb_ready_flag&0x04	&& ms_cnt%5==2)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                Set_AGV_Velocity_Vector_Data_Update(AGV_C_Tx_Data, chassis.C_motor.target_angle, chassis.C_motor.target_speed.output, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY);       
            }
            else
            {
							  Set_AGV_Velocity_Vector_Data_Update(AGV_C_Tx_Data, chassis.C_motor.target_angle, chassis.C_motor.target_speed.output, chassis_power_control.scaled_power_32[2]);   
            }
        }
        else
        {
								Set_AGV_Velocity_Vector_Data_Update(AGV_C_Tx_Data, chassis.C_motor.target_angle, 0, chassis_power_control.scaled_power_32[2]);   
        }
				CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.C_motor.ID,0,0x03),AGV_C_Tx_Data,8);
        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x0B;
    }
		if(chassis_power_control.all_mscb_ready_flag&0x08	&& ms_cnt%5==3)
    {
        if (chassis.parameter.mode != CHASSIS_REMOTE_CLOSE)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                Set_AGV_Velocity_Vector_Data_Update(AGV_D_Tx_Data, chassis.D_motor.target_angle, chassis.A_motor.target_speed.output, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY);       
            }
            else
            {
							  Set_AGV_Velocity_Vector_Data_Update(AGV_D_Tx_Data, chassis.D_motor.target_angle, chassis.A_motor.target_speed.output, chassis_power_control.scaled_power_32[3]);   
            }
        }
        else
        {
								Set_AGV_Velocity_Vector_Data_Update(AGV_D_Tx_Data, chassis.D_motor.target_angle, 0, chassis_power_control.scaled_power_32[3]);   
        }

        CAN_Send_EXT_Data(&hcan2,EXT_ID_Set(chassis.D_motor.ID,0,0x03),AGV_D_Tx_Data,8);
				chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x07;
    }

	
}



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