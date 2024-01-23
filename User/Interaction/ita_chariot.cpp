/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
    DR16_Dead_Zone = __DR16_Dead_Zone;

    //裁判系统
    Referee.Init(&huart6);

    //遥控器
    DR16.Init(&huart1);

    //迷你主机
    MiniPC.Init();

    //底盘
    Chassis.Referee = &Referee;
    Chassis.Init(1.0f, 1.0f, 2.0f);

    //云台
    Gimbal.MiniPC = &MiniPC;
    Gimbal.Init();

    //发射机构
    Booster.Referee = &Referee;
    Booster.Init();
}

/**
 * @brief 底盘控制逻辑
 *
 */
void Class_Chariot::Control_Chassis()
{
    //速度目标值
    float tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_omega = 0;
    //遥控器摇杆值
    float dr16_l_x, dr16_l_y, dr16_r_x;

    //排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
    dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    {
        //遥控器离线或下方失能
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

        return;
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)
    {
        //中间遥控模式
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ABSOLUTE);

        //遥控器操作逻辑

        //设定矩形到圆形映射进行控制
        tmp_chassis_velocity_x = dr16_l_x * sqrt(1 - dr16_l_y * dr16_l_y / 2) * Chassis.Get_Velocity_X_Max() * Chassis.Get_Velocity_X_Max();
        tmp_chassis_velocity_y = dr16_l_y * sqrt(1 - dr16_l_x * dr16_l_x / 2) * Chassis.Get_Velocity_Y_Max() * Chassis.Get_Velocity_Y_Max();
        tmp_chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();

        //键盘遥控器操作逻辑
        if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED)
        {
            tmp_chassis_velocity_x -= Chassis.Get_Velocity_X_Max();
        }
        if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
        {
            tmp_chassis_velocity_x += Chassis.Get_Velocity_X_Max();
        }
        if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED)
        {
            tmp_chassis_velocity_y += Chassis.Get_Velocity_Y_Max();
        }
        if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
        {
            tmp_chassis_velocity_y -= Chassis.Get_Velocity_Y_Max();
        }
        if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED)
        {
            tmp_chassis_omega += Chassis.Get_Omega_Max();
        }
        if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_PRESSED)
        {
            tmp_chassis_omega -= Chassis.Get_Omega_Max();
        }
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP)
    {
        //上方导航模式
        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
        {
            //迷你主机离线
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        else
        {
            //迷你主机在线
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ABSOLUTE);

            //迷你主机操作逻辑
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ABSOLUTE);
            tmp_chassis_velocity_x = MiniPC.Get_Chassis_Target_Velocity_X();
            tmp_chassis_velocity_y = MiniPC.Get_Chassis_Target_Velocity_Y();
            tmp_chassis_omega = MiniPC.Get_Chassis_Target_Omega();
        }
    }

    // 设定速度
    Chassis.Set_Target_Velocity_X(tmp_chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(tmp_chassis_velocity_y);
    Chassis.Set_Target_Omega(tmp_chassis_omega);
}

/**
 * @brief 云台控制逻辑
 *
 */
void Class_Chariot::Control_Gimbal()
{
    //角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch;
    //遥控器摇杆值
    float dr16_y, dr16_r_y;

    //获取当前角度值
    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();

    // 排除遥控器死区
    dr16_y = (Math_Abs(DR16.Get_Yaw()) > DR16_Dead_Zone) ? DR16.Get_Yaw() : 0;
    dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    {
        //遥控器离线或下方失能
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)
    {
        //中间遥控模式
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

        //遥控器操作逻辑

        tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Resolution;
        tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Resolution;

        //键盘遥控器操作逻辑

        tmp_gimbal_yaw += DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
        tmp_gimbal_pitch -= DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP)
    {
        //上方导航模式

        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
        {
            //迷你主机离线
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        }
        else
        {
            //迷你主机在线
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

            //迷你主机操作逻辑

            tmp_gimbal_yaw += MiniPC_Autoaiming_Yaw_Angle_Resolution * Gimbal.MiniPC->Get_Gimbal_Target_Yaw_Angle();
            tmp_gimbal_pitch += MiniPC_Autoaiming_Pitch_Angle_Resolution * Gimbal.MiniPC->Get_Gimbal_Target_Pitch_Angle();
        }
    }

    // 设定角度
    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
}

/**
 * @brief 发射机构控制逻辑
 *
 */
void Class_Chariot::Control_Booster()
{

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    {
        //遥控器离线或下方失能
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);

        return;
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)
    {
        //中间遥控模式

        // 遥控器键盘鼠标操作逻辑

        if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_UP || DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN || DR16.Get_Mouse_Left_Key() == DR16_Key_Status_PRESSED)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE || DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_PRESSED_FREE)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
        }
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP)
    {
        //上方导航模式
        if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
        {
            //迷你主机离线
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        }
        else
        {
            //迷你主机在线
            Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);

            //迷你主机操作逻辑

            Booster.Set_Driver_Omega(MiniPC.Get_Booster_Frequency() * PI / 4.0f);
        }

        //根据遥控器判断自家颜色, 上红下蓝
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            MiniPC.Set_Self_Color(MiniPC_Self_Color_RED);
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
        {
            MiniPC.Set_Self_Color(MiniPC_Self_Color_BLUE);
        }
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_TRIG_UP_MIDDLE)
    {
        //由迷你主机切换回遥控

        Booster.Set_Driver_Omega(Booster.Get_Default_Driver_Omega());

        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
    }
}

/**
 * @brief 控制回调函数
 *
 */
void Class_Chariot::TIM_Control_Callback()
{
    //底盘云台发射机构的控制策略
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();

    //迷你主机相关数据的设置
    MiniPC.Set_Game_Stage(static_cast<Enum_MiniPC_Game_Stage>(Referee.Get_Game_Stage()));
    MiniPC.Set_Gimbal_Now_Yaw_Angle(Gimbal.Motor_Yaw.Get_Now_Angle());
    if (Gimbal.WIT.Get_WIT_Status() == WIT_Status_DISABLE)
    {
        MiniPC.Set_Gimbal_Now_Yaw_Omega(Gimbal.Motor_Yaw.Get_Now_Omega());
    }
    else
    {
        MiniPC.Set_Gimbal_Now_Yaw_Omega(Gimbal.WIT.Get_Omega_Z());
    }
    MiniPC.Set_Gimbal_Now_Pitch_Angle(Gimbal.Motor_Pitch.Get_Now_Angle());
    if (Gimbal.WIT.Get_WIT_Status() == WIT_Status_DISABLE)
    {
        MiniPC.Set_Gimbal_Now_Pitch_Omega(Gimbal.Motor_Pitch.Get_Now_Omega());
    }
    else
    {
        MiniPC.Set_Gimbal_Now_Pitch_Omega(Gimbal.WIT.Get_Omega_Y());
    }
    MiniPC.Set_Armor_Attacked_ID(Referee.Get_Armor_Attacked_ID());
    if (Referee.Get_Attacked_Type() == Referee_Data_Robot_Damage_Type_ARMOR_ATTACKED)
    {
        MiniPC.Set_Armor_Attacked_Ammo_Status(MiniPC_Data_Status_ENABLE);
    }
    else
    {
        MiniPC.Set_Armor_Attacked_Ammo_Status(MiniPC_Data_Status_DISABLE);
    }
    if (Referee.Get_Attacked_Type() == Referee_Data_Robot_Damage_Type_ARMOR_ATTACKED)
    {
        MiniPC.Set_Armor_Attacked_Ammo_Status(MiniPC_Data_Status_ENABLE);
    }
    else
    {
        MiniPC.Set_Armor_Attacked_Ammo_Status(MiniPC_Data_Status_DISABLE);
    }
    if (Referee.Get_Event_Outpost_Status() == Referee_Data_Status_ENABLE)
    {
        MiniPC.Set_Outpost_Status(MiniPC_Data_Status_ENABLE);
    }
    else
    {
        MiniPC.Set_Outpost_Status(MiniPC_Data_Status_DISABLE);
    }
    if (Referee.Get_Remaining_Time() <= 240)
    {
        MiniPC.Set_Outpost_Protect_Status(MiniPC_Data_Status_DISABLE);
    }
    else
    {
        MiniPC.Set_Outpost_Protect_Status(MiniPC_Data_Status_ENABLE);
    }

    //各个模块的分别解算
    Chassis.TIM_Calculate_PeriodElapsedCallback();
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    Booster.TIM_Calculate_PeriodElapsedCallback();
    MiniPC.TIM_Write_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
