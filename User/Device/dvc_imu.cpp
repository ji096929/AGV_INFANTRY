//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "dvc_imu.h"


void Class_IMU::Init()
{
    // 初始化BMI088传感器，并检查初始化是否成功
    IMU_BMI088.init(&hspi1);
    HAL_Delay(100);

    // 初始化IST8310传感器
    IMU_IST8310.init(&hi2c3);
    HAL_Delay(100);

    // 初始化MahonyAHRS算法，并传入初始四元数
    IMU_MahonyAHRS.init(INS_Quat);
 
    //初始化温控pid参数
    PID_IMU_Tempture.Init(2000, 3000, 0, 0.0, uint32_max, uint32_max);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

}


void Class_IMU::TIM_Calculate_PeriodElapsedCallback(void)
{
    static uint8_t Tempture_Cnt_mod50 = 0;
    Tempture_Cnt_mod50++;

    IMU_BMI088.BMI088_Read(&BMI088_Raw_Data);
        
    //加速度计低通滤波
    //accel low-pass filter
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + BMI088_Raw_Data.Accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + BMI088_Raw_Data.Accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + BMI088_Raw_Data.Accel[2] * fliter_num[2];

    //角速度零值修正，目的是建立死区，mpu波纹浮动
    BMI088_Raw_Data.Gyro[0] = fabs(BMI088_Raw_Data.Gyro[0])<GIMBAL_GYRO_X_ZERO_CORRECT ? 0.0f : BMI088_Raw_Data.Gyro[0];
    BMI088_Raw_Data.Gyro[1] = fabs(BMI088_Raw_Data.Gyro[1])<GIMBAL_GYRO_Y_ZERO_CORRECT ? 0.0f : BMI088_Raw_Data.Gyro[1];
    BMI088_Raw_Data.Gyro[2] = fabs(BMI088_Raw_Data.Gyro[2])<GIMBAL_GYRO_Z_ZERO_CORRECT ? 0.0f : BMI088_Raw_Data.Gyro[2];
    
//  IMU_IST8310.ist8310_read_mag(IST8310_Real_Data.mag);

    IMU_MahonyAHRS.AHRS_update(INS_Quat, 0.001f, BMI088_Raw_Data.Gyro, BMI088_Raw_Data.Accel, IST8310_Real_Data.mag);
    Get_Angle();

    if(Tempture_Cnt_mod50 % 50 == 0)
    {
        PID_IMU_Tempture.Set_Now(BMI088_Raw_Data.Temperature);
        PID_IMU_Tempture.Set_Target(45.0f);
        PID_IMU_Tempture.TIM_Adjust_PeriodElapsedCallback();
        TIM_Set_PWM(&htim10, TIM_CHANNEL_1, (uint16_t)PID_IMU_Tempture.Get_Out());
    }

    imu_start_flag = 1;
}

void Class_IMU::TIM1msMod50_Alive_PeriodElapsedCallback(void)
{
    if(imu_start_flag)  //陀螺仪已经开启通讯
    {
        if((Pre_BMI088_Raw_Data.Accel[0] == BMI088_Raw_Data.Accel[0]) 
        && (Pre_BMI088_Raw_Data.Accel[1] == BMI088_Raw_Data.Accel[1]) 
        && (Pre_BMI088_Raw_Data.Accel[2] == BMI088_Raw_Data.Accel[2]))  //判断陀螺仪是否掉线
        {
            IMU_Status = IMU_Status_DISABLE;
        }
        else IMU_Status = IMU_Status_ENABLE;

        memcpy(&Pre_BMI088_Raw_Data, &BMI088_Raw_Data, sizeof(IMU_Data_t));  //保留上一次的数据
    }
}

void Class_IMU::Get_Angle()
{
    INS_Rad[0] = atan2f(2.0f*(INS_Quat[0]*INS_Quat[3]+INS_Quat[1]*INS_Quat[2]), 2.0f*(INS_Quat[0]*INS_Quat[0]+INS_Quat[1]*INS_Quat[1])-1.0f);
    INS_Rad[1] = asinf(-2.0f*(INS_Quat[1]*INS_Quat[3]-INS_Quat[0]*INS_Quat[2]));
    INS_Rad[2] = atan2f(2.0f*(INS_Quat[0]*INS_Quat[1]+INS_Quat[2]*INS_Quat[3]),2.0f*(INS_Quat[0]*INS_Quat[0]+INS_Quat[3]*INS_Quat[3])-1.0f);
    for(int i=0;i<3;i++){
        INS_Angle[i] = INS_Rad[i] * 180.0f / 3.1415926f;
    }
}

void Class_IMU::TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;
    if(value < 0) value = 0;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}

float Class_IMU::Get_Angle_Roll(void)
{
    return INS_Angle[0];
}

float Class_IMU::Get_Angle_Pitch(void)
{
    return INS_Angle[1];
}

float Class_IMU::Get_Angle_Yaw(void)
{
    return INS_Angle[2];
}

float Class_IMU::Get_Accel_X(void)
{
    return BMI088_Raw_Data.Accel[0];
}

float Class_IMU::Get_Accel_Y(void)
{
    return BMI088_Raw_Data.Accel[1];
}

float Class_IMU::Get_Accel_Z(void)
{
    return BMI088_Raw_Data.Accel[2];
}

float Class_IMU::Get_Gyro_Roll(void)
{
    return BMI088_Raw_Data.Gyro[0];
}

float Class_IMU::Get_Gyro_Pitch(void)
{
    return BMI088_Raw_Data.Gyro[1];
}

float Class_IMU::Get_Gyro_Yaw(void)
{
    return BMI088_Raw_Data.Gyro[2];
}

float Class_IMU::Get_Rad_Roll(void)
{
    return INS_Rad[0];
}

float Class_IMU::Get_Rad_Pitch(void)
{
    return INS_Rad[1];
}

float Class_IMU::Get_Rad_Yaw(void)
{
    return INS_Rad[2];
}

Enum_IMU_Status Class_IMU::Get_IMU_Status(void)
{
    return IMU_Status;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

