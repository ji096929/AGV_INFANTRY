/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega);
        if (WIT->Get_WIT_Status() == WIT_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Set_K_P(1500.0f);
            PID_Omega.Set_K_I(3000.0f);
            PID_Omega.Set_K_D(0.15f);
            PID_Torque.Set_K_P(2.0f);
            PID_Torque.Set_K_I(15.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Omega.Set_Now(WIT->Get_Omega_Z());
            PID_Omega.Set_K_P(10000.0f);
            PID_Omega.Set_K_I(100000.0f);
            PID_Omega.Set_K_D(0.1f);
            PID_Torque.Set_K_P(0.78f);
            PID_Torque.Set_K_I(100.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle);

        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega);
        if (WIT->Get_WIT_Status() == WIT_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Angle.Set_K_P(7.0f);
            PID_Angle.Set_K_I(0.0f);
            PID_Angle.Set_K_D(0.1f);
            PID_Omega.Set_K_P(1500.0f);
            PID_Omega.Set_K_I(3000.0f);
            PID_Omega.Set_K_D(0.15f);
            PID_Torque.Set_K_P(2.0f);
            PID_Torque.Set_K_I(15.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Omega.Set_Now(-WIT->Get_Omega_Z());
            PID_Angle.Set_K_P(28.0f);
            PID_Angle.Set_K_I(0.0f);
            PID_Angle.Set_K_D(0.03f);
            PID_Omega.Set_K_P(10000.0f);
            PID_Omega.Set_K_I(100000.0f);
            PID_Omega.Set_K_D(0.1f);
            PID_Torque.Set_K_P(0.78f);
            PID_Torque.Set_K_I(100.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega);
        if (WIT->Get_WIT_Status() == WIT_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Set_K_P(1000.0f);
            PID_Omega.Set_K_I(5000.0f);
            PID_Omega.Set_K_D(0.7f);
            PID_Torque.Set_K_P(2.0f);
            PID_Torque.Set_K_I(15.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Omega.Set_Now(WIT->Get_Omega_Y());
            PID_Omega.Set_K_P(8000.0f);
            PID_Omega.Set_K_I(100000.0f);
            PID_Omega.Set_K_D(0.0003f);
            PID_Torque.Set_K_P(0.8f);
            PID_Torque.Set_K_I(100.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle);

        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega);
        if (WIT->Get_WIT_Status() == WIT_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Angle.Set_K_P(7.0f);
            PID_Angle.Set_K_I(0.0f);
            PID_Angle.Set_K_D(0.1f);
            PID_Omega.Set_K_P(1000.0f);
            PID_Omega.Set_K_I(5000.0f);
            PID_Omega.Set_K_D(0.7f);
            PID_Torque.Set_K_P(2.0f);
            PID_Torque.Set_K_I(15.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        else
        {
            PID_Omega.Set_Now(WIT->Get_Omega_Y());
            PID_Angle.Set_K_P(16.5f);
            PID_Angle.Set_K_I(0.0f);
            PID_Angle.Set_K_D(0.01f);
            PID_Omega.Set_K_P(8000.0f);
            PID_Omega.Set_K_I(100000.0f);
            PID_Omega.Set_K_D(0.0003f);
            PID_Torque.Set_K_P(0.8f);
            PID_Torque.Set_K_I(100.0f);
            PID_Torque.Set_K_D(0.0f);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out() + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    //云台AHRS
    WIT.Init(&huart7);

    //yaw轴电机
    Motor_Yaw.PID_Angle.Init(28.0f, 0.0f, 0.03f, 0.0f, 6.0f * PI, 6.0f * PI);
    Motor_Yaw.PID_Omega.Init(10000.0f, 100000.0f, 0.1f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.WIT = &WIT;
    Motor_Yaw.MiniPC = MiniPC;
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);

    //pitch轴电机
    Motor_Pitch.PID_Angle.Init(16.5f, 0.0f, 0.01f, 0.0f, 6.0f * PI, 6.0f * PI);
    Motor_Pitch.PID_Omega.Init(8000.0f, 100000.0f, 0.0003f, 0, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.WIT = &WIT;
    Motor_Pitch.Init(&hcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);
}

/**
 * @brief 输出到电机
 *
 */
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        //云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Omega(0.0f);
        Motor_Pitch.Set_Target_Omega(0.0f);
    }
    else if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
    {
        //云台工作
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
