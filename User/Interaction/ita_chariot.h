/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef TSK_INTERACTION_H
#define TSK_INTERACTION_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"
#include "crt_chassis.h"
#include "crt_gimbal.h"
#include "crt_booster.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
    //裁判系统
    Class_Referee Referee;
    //遥控器
    Class_DR16 DR16;
    //迷你主机
    Class_MiniPC MiniPC;

    //底盘
    Class_Tricycle_Chassis Chassis;
    //云台
    Class_Gimbal Gimbal;
    //发射机构
    Class_Booster Booster;

    void Init(float __DR16_Dead_Zone = 0);

    void TIM_Control_Callback();

protected:
    //初始化相关常量

    //遥控器拨动的死区, 0~1
    float DR16_Dead_Zone;

    //常量

    //DR16底盘加速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
    float DR16_Keyboard_Chassis_Speed_Resolution_Small = 0.001f;
    //DR16底盘减速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
    float DR16_Keyboard_Chassis_Speed_Resolution_Big = 0.01f;

    //DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
    float DR16_Yaw_Resolution = 0.0005f * PI;
    //DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
    float DR16_Pitch_Resolution = 0.0005f * PI;

    //DR16鼠标云台yaw灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Yaw_Angle_Resolution = 1.0f;
    //DR16鼠标云台pitch灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Pitch_Angle_Resolution = 1.0f;
    
    //迷你主机云台pitch自瞄控制系数
    float MiniPC_Autoaiming_Yaw_Angle_Resolution = 0.003f;
    //迷你主机云台pitch自瞄控制系数
    float MiniPC_Autoaiming_Pitch_Angle_Resolution = 0.003f;

    //内部变量

    //读变量

    //写变量

    //读写变量

    //内部函数

    void Control_Chassis();
    void Control_Gimbal();
    void Control_Booster();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
