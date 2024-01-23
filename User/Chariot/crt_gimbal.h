/**
 * @file crt_gimbal.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_witahrs.h"
#include "dvc_minipc.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_GYROSCOPE,
};

/**
 * @brief Specialized, yaw轴电机类
 *
 */
class Class_Gimbal_Yaw_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    //陀螺仪获取云台角速度
    Class_WIT *WIT;
    //迷你主机获取底盘角速度补偿
    Class_MiniPC *MiniPC;

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量

    //内部变量

    //读变量

    //写变量

    //读写变量

    //内部函数
};

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    Class_WIT *WIT;

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //常量

    float Gravity_Compensate = 1700.0f;

    //内部变量

    //读变量

    //写变量

    //读写变量

    //内部函数
};

/**
 * @brief Specialized, 云台类
 *
 */
class Class_Gimbal
{
public:
    //云台AHRS
    Class_WIT WIT;

    //迷你主机
    Class_MiniPC *MiniPC;

    // yaw轴电机
    Class_Gimbal_Yaw_Motor_GM6020 Motor_Yaw;
    // pitch轴电机
    Class_Gimbal_Pitch_Motor_GM6020 Motor_Pitch;

    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量

    // yaw轴最小值
    float Min_Yaw_Angle = -PI / 5.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = PI / 5.0f;
    // pitch轴最小值
    float Min_Pitch_Angle = -0.12f;
    // pitch轴最大值
    float Max_Pitch_Angle = 0.20f;

    //内部变量

    //读变量

    //写变量

    //云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE;

    //读写变量

    // yaw轴角度
    float Target_Yaw_Angle = 0.0f;
    // pitch轴角度
    float Target_Pitch_Angle = 0.0f;

    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}

/**
 * @brief 获取pitch轴角度
 *
 * @return float pitch轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}

/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
}

/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
