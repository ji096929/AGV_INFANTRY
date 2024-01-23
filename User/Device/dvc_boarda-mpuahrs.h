/**
 * @file dvc_boarda-mpuahrs.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief A板自带MPU姿态传感器
 * @version 0.1
 * @date 2023-09-29 0.1 新增A板MPU库
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_BOARDA_MPUAHRS_H
#define DVC_BOARDA_MPUAHRS_H

/* Includes ------------------------------------------------------------------*/

#include "string.h"
#include "drv_bsp-boarda.h"
#include "drv_spi.h"
#include "alg_pid.h"
#include "alg_filter.h"
#include "dvc_boarda-mpuahrs-register.h"
#include "dvc_boarda-mpuahrs-istmagnet.h"

/* Exported macros -----------------------------------------------------------*/

//MPU6500中, SPI读数据需要的置位, 写数据不需要
#define MPU6500_READ_MASK (0x80)

//MPU6500中, 自身固定值, 位于WHO_AM_I寄存器
#define MPU6500_ID (0x70)

//加速度计量程与量纲对应关系, 陀螺仪返回加速度值范围是±32768, 当量程为±4g时, 该值为4
#define MPU6500_ACCEL_RANGE (4.0f)
//加速度计量程与量纲对应关系, 陀螺仪返回加速度值范围是±32768, 当量程为±2000dps时, 改值为2000
#define MPU6500_GYRO_RANGE (2000.0f)

//默认计算周期, 尽量别动
#define MPU6500_CALCULATE_PERIOD (0.001f)
#define MPU6500_CALCULATE_PERIOD_HALF (MPU6500_CALCULATE_PERIOD / 2.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MPU姿态传感器状态
 *
 */
enum Enum_MPU_Status
{
    MPU_Status_DISABLE = 0,
    MPU_Status_ENABLE,
};

/**
 * @brief MPU姿态传感器经过处理的数据, 加速度
 * 
 */
struct Struct_BoardA_MPU_Data_Accelerate
{
    float Accelerate_X;
    float Accelerate_Y;
    float Accelerate_Z;
};

/**
 * @brief MPU姿态传感器经过处理的数据, 角速度
 * 
 */
struct Struct_BoardA_MPU_Data_Omega
{
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
};

/**
 * @brief MPU姿态传感器经过处理的数据, 磁场强度
 * 
 */
struct Struct_BoardA_MPU_Data_Magnet
{
    float Magnet_X;
    float Magnet_Y;
    float Magnet_Z;
};

/**
 * @brief MPU姿态传感器经过处理的数据, 角度, 以Yaw-Pitch-Roll顺序旋转
 * 
 */
struct Struct_BoardA_MPU_Data_Angle
{
    float Angle_Yaw;
    float Angle_Pitch;
    float Angle_Roll;
};

/**
 * @brief MPU姿态传感器经过处理的数据, 四元数
 * 
 */
struct Struct_BoardA_MPU_Data_Quaternion
{
    float q_0;
    float q_1;
    float q_2;
    float q_3;
};

/**
 * @brief Generic, A板姿态传感器, 正对R标平放地面, 六轴模式初始右前上, 九轴模式东北天
 * 
 */
class Class_BoardA_MPU
{
public:	
    //Mahony算法专属PID
    Class_PID PID_Mahony_X;
    Class_PID PID_Mahony_Y;
    Class_PID PID_Mahony_Z;

    //滤波器算法
    Class_Filter_Fourier Filter_Fourier_Accelerate_X;
    Class_Filter_Fourier Filter_Fourier_Accelerate_Y;
    Class_Filter_Fourier Filter_Fourier_Accelerate_Z;
    Class_Filter_Fourier Filter_Fourier_Temperature;
    Class_Filter_Fourier Filter_Fourier_Omega_X;
    Class_Filter_Fourier Filter_Fourier_Omega_Y;
    Class_Filter_Fourier Filter_Fourier_Omega_Z;
    Class_Filter_Fourier Filter_Fourier_Magnet_X;
    Class_Filter_Fourier Filter_Fourier_Magnet_Y;
    Class_Filter_Fourier Filter_Fourier_Magnet_Z;

    void Init();

    inline float Get_Accelerate_X();
    inline float Get_Accelerate_Y();
    inline float Get_Accelerate_Z();
    inline float Get_Temperature();
    inline float Get_Omega_X();
    inline float Get_Omega_Y();
    inline float Get_Omega_Z();
    inline float Get_Magnet_X();
    inline float Get_Magnet_Y();
    inline float Get_Magnet_Z();
    inline float Get_Angle_Yaw();
    inline float Get_Angle_Pitch();
    inline float Get_Angle_Roll();
    inline float Get_Quaternion_0();
    inline float Get_Quaternion_1();
    inline float Get_Quaternion_2();
    inline float Get_Quaternion_3();
    
    void SPI_TxRxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer);
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM100us_Send_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;
    //片选的GPIO
    GPIO_TypeDef* CS_GPIOx;
    uint16_t CS_GPIO_Pin;

    //常量

    //内部变量

    //1000us定时周期分频器
    int TIM100us_Send_PeriodElapsedCallback_Mod10 = 0;
    //MPU姿态传感器在线状态
    Enum_MPU_Status MPU_Status = MPU_Status_DISABLE;

    //陀螺仪原数据偏置
    float Gyro_X_Offset = 0.0f;
    float Gyro_Y_Offset = 0.0f;
    float Gyro_Z_Offset = 0.0f;
    //加速度计原数据偏置
    float Accel_X_Offset = 0.0f;
    float Accel_Y_Offset = 0.0f;
    float Accel_Z_Offset = 0.0f;

    //MPU姿态传感器接收的原始数据
    Struct_BoardA_MPU_Data_Accelerate Raw_Data_Accelerate;
    float Raw_Temperature;
    Struct_BoardA_MPU_Data_Omega Raw_Data_Omega;
    Struct_BoardA_MPU_Data_Magnet Raw_Data_Magnet;

    //就近转位防止跳变
    Struct_BoardA_MPU_Data_Angle Pre_Angle;

    //读变量

    //MPU姿态传感器对外接口信息
    Struct_BoardA_MPU_Data_Accelerate Data_Accelerate;
    float Temperature = 0.0f;
    Struct_BoardA_MPU_Data_Omega Data_Omega;
    Struct_BoardA_MPU_Data_Magnet Data_Magnet;
    Struct_BoardA_MPU_Data_Angle Data_Angle;
    Struct_BoardA_MPU_Data_Quaternion Data_Quaternion;

    //写变量

    //读写变量

    //内部函数
    
    uint8_t IST_Register_Read(uint8_t Address);
    void IST_Register_Write(uint8_t Address, uint8_t Data);
    void MPU_IMU_Config();
    void MPU_I2C_Master_Config();
    void MPU_I2C_Slave_Config();
    void MPU_IST_Register_Config();
    void MPU_IST_Auto_Communicate_Config();
    void Register_Init();
    
    void MPU_Offset_Config();

    void Pose_Calculation();

    void Data_Process();
};
 
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取加速度x
 * 
 * @return float 加速度x
 */
float Class_BoardA_MPU::Get_Accelerate_X()
{
    return(Data_Accelerate.Accelerate_X);
}

/**
 * @brief 获取加速度y
 * 
 * @return float 加速度y
 */
float Class_BoardA_MPU::Get_Accelerate_Y()
{
    return(Data_Accelerate.Accelerate_Y);
}

/**
 * @brief 获取加速度z
 * 
 * @return float 加速度z
 */
float Class_BoardA_MPU::Get_Accelerate_Z()
{
    return(Data_Accelerate.Accelerate_Z);
}

/**
 * @brief 获取温度
 * 
 * @return float 温度
 */
float Class_BoardA_MPU::Get_Temperature()
{
    return(Temperature);
}

/**
 * @brief 获取角速度x
 * 
 * @return float 角速度x
 */
float Class_BoardA_MPU::Get_Omega_X()
{
    return(Data_Omega.Omega_X);
}

/**
 * @brief 获取角速度y
 * 
 * @return float 角速度y
 */
float Class_BoardA_MPU::Get_Omega_Y()
{
    return(Data_Omega.Omega_Y);
}

/**
 * @brief 获取角速度z
 * 
 * @return float 角速度z
 */
float Class_BoardA_MPU::Get_Omega_Z()
{
    return(Data_Omega.Omega_Z);
}

/**
 * @brief 获取磁场强度x
 * 
 * @return float 磁场强度x
 */
float Class_BoardA_MPU::Get_Magnet_X()
{
    return(Data_Magnet.Magnet_X);
}

/**
 * @brief 获取磁场强度y
 * 
 * @return float 磁场强度y
 */
float Class_BoardA_MPU::Get_Magnet_Y()
{
    return(Data_Magnet.Magnet_Y);
}

/**
 * @brief 获取磁场强度z
 * 
 * @return float 磁场强度z
 */
float Class_BoardA_MPU::Get_Magnet_Z()
{
    return(Data_Magnet.Magnet_Z);
}

/**
 * @brief 获取航向
 * 
 * @return float 航向角
 */
float Class_BoardA_MPU::Get_Angle_Yaw()
{
    return(Data_Angle.Angle_Yaw);
}

/**
 * @brief 获取俯仰角
 * 
 * @return float 俯仰角
 */
float Class_BoardA_MPU::Get_Angle_Pitch()
{
    return(Data_Angle.Angle_Pitch);
}

/**
 * @brief 获取滚转角
 * 
 * @return float 滚转角
 */
float Class_BoardA_MPU::Get_Angle_Roll()
{
    return(Data_Angle.Angle_Roll);
}

/**
 * @brief 获取四元数Q0
 * 
 * @return float 四元数Q0
 */
float Class_BoardA_MPU::Get_Quaternion_0()
{
    return(Data_Quaternion.q_0);
}

/**
 * @brief 获取四元数Q1
 * 
 * @return float 四元数Q1
 */
float Class_BoardA_MPU::Get_Quaternion_1()
{
    return(Data_Quaternion.q_1);
}

/**
 * @brief 获取四元数Q2
 * 
 * @return float 四元数Q2
 */
float Class_BoardA_MPU::Get_Quaternion_2()
{
    return(Data_Quaternion.q_2);
}

/**
 * @brief 获取四元数Q3
 * 
 * @return float 四元数Q3
 */
float Class_BoardA_MPU::Get_Quaternion_3()
{
    return(Data_Quaternion.q_3);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
