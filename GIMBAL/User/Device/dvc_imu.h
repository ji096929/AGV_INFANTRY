//=====================================================================================================
// MahonyAHRS.h
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
#ifndef DVC_IMU_H
#define DVC_IMU_H

#include "main.h"
#include "dvc_boardc_bmi088.h"
#include "dvc_boardc_ist8310.h"
#include "alg_MahonyAHRS.h"
#include "alg_pid.h"

//----------------------------------------------------------------------------------------------------

enum Enum_IMU_Status 
{
    IMU_Status_DISABLE = 0,
    IMU_Status_ENABLE,
};

// Variable declaration

class Class_IMU
{
    public:

    void Init(void);
    void TIM_Calculate_PeriodElapsedCallback(void);
    void TIM1msMod50_Alive_PeriodElapsedCallback(void);

    float Get_Angle_Roll(void);
    float Get_Angle_Pitch(void);
    float Get_Angle_Yaw(void);

    float Get_Rad_Roll(void);
    float Get_Rad_Pitch(void);
    float Get_Rad_Yaw(void);

    float Get_Accel_X(void);
    float Get_Accel_Y(void);
    float Get_Accel_Z(void);

    float Get_Gyro_Roll(void);
    float Get_Gyro_Pitch(void);
    float Get_Gyro_Yaw(void);

    Enum_IMU_Status Get_IMU_Status(void);

    protected:

    Class_MahonyAHRS IMU_MahonyAHRS;
    Class_BoardC_BMI IMU_BMI088;
    Class_BoardC_IST8310 IMU_IST8310;
    Class_PID PID_IMU_Tempture;
    
    Enum_IMU_Status IMU_Status;

    void Get_Angle(void);
    void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

    IMU_Data_t BMI088_Raw_Data;
    IMU_Data_t Pre_BMI088_Raw_Data;
    IST8310_Real_Data_t IST8310_Real_Data;
			
    float INS_Quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float INS_Rad[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
    float INS_Angle[3] = {0.0f, 0.0f, 0.0f};
		
    uint8_t gyro_update_flag = 0;
    uint8_t accel_update_flag = 0;
    uint8_t accel_temp_update_flag = 0;
    uint8_t mag_update_flag = 0;
    uint8_t imu_start_flag = 0;    

    const float GIMBAL_GYRO_X_ZERO_CORRECT = 0.05f;                     //俯仰陀螺仪x轴方向死区修正(云台)
    const float  GIMBAL_GYRO_Y_ZERO_CORRECT = 0.05f;                       //俯仰陀螺仪y轴方向死区修正(云台)
    const float  GIMBAL_GYRO_Z_ZERO_CORRECT = 0.05f;                        //俯仰陀螺仪z轴方向死区修正(云台)
    
    //加速度计低通滤波
    float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
    const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
};

//---------------------------------------------------------------------------------------------------
// Function declarations



#endif
//=====================================================================================================
// End of file
//=====================================================================================================
