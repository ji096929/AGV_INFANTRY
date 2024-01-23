/**
 * @file crt_booster.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 发射机构电控
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 摩擦轮编号
 * 1 2
 */

#ifndef CRT_BOOSTER_H
#define CRT_BOOSTER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Class_Booster;

/**
 * @brief 发射机构控制类型
 *
 */
enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,
    Booster_Control_Type_CEASEFIRE,
    Booster_Control_Type_SINGLE,
    Booster_Control_Type_REPEATED,
};

/**
 * @brief Specialized, 热量检测有限自动机
 *
 */
class Class_FSM_Heat_Detect : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Heat;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 卡弹策略有限自动机
 *
 */
class Class_FSM_Antijamming : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:
    //热量检测有限自动机
    Class_FSM_Heat_Detect FSM_Heat_Detect;
    friend class Class_FSM_Heat_Detect;

    //卡弹策略有限自动机
    Class_FSM_Antijamming FSM_Antijamming;
    friend class Class_FSM_Antijamming;

    //裁判系统
    Class_Referee *Referee;

    //拨弹盘电机
    Class_DJI_Motor_C610 Motor_Driver;

    //摩擦轮电机左
    Class_DJI_Motor_C620 Motor_Friction_Left;
    //摩擦轮电机右
    Class_DJI_Motor_C620 Motor_Friction_Right;

    void Init();

    inline float Get_Default_Driver_Omega();

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    inline void Set_Friction_Omega(float __Friction_Omega);
    inline void Set_Driver_Omega(float __Driver_Omega);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量

    //拨弹盘堵转扭矩阈值, 超出被认为卡弹
    uint16_t Driver_Torque_Threshold = 9000;
    //摩擦轮单次判定发弹阈值, 超出被认为发射子弹
    uint16_t Friction_Torque_Threshold = 2200;
    //摩擦轮速度判定发弹阈值, 超出则说明已经开机
    uint16_t Friction_Omega_Threshold = 695;

    //内部变量

    //读变量

    //拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
    float Default_Driver_Omega = 2.0f * PI;

    //写变量

    //发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    //摩擦轮角速度
    float Friction_Omega = 700.0f;
    //拨弹盘实际的目标速度, 一圈八发子弹
    float Driver_Omega = 2.0f * PI;

    //读写变量

    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
 *
 * @return float 拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
 */
float Class_Booster::Get_Default_Driver_Omega()
{
    return (Default_Driver_Omega);
}

/**
 * @brief 设定发射机构状态
 *
 * @param __Booster_Control_Type 发射机构状态
 */
void Class_Booster::Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief 设定摩擦轮角速度
 *
 * @param __Friction_Omega 摩擦轮角速度
 */
void Class_Booster::Set_Friction_Omega(float __Friction_Omega)
{
    Friction_Omega = __Friction_Omega;
}

/**
 * @brief 设定拨弹盘实际的目标速度, 一圈八发子弹
 *
 * @param __Driver_Omega 拨弹盘实际的目标速度, 一圈八发子弹
 */
void Class_Booster::Set_Driver_Omega(float __Driver_Omega)
{
    Driver_Omega = __Driver_Omega;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
