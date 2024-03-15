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
    #ifdef CHASSIS
    
        //裁判系统
        Referee.Init(&huart6);

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init(10.0f, 10.0f, 2.0f);

    #elif defined(GIMBAL)

        //遥控器
        DR16.Init(&huart3);
        DR16_Dead_Zone = __DR16_Dead_Zone;   

        //云台
        Gimbal.Init();

        //发射机构
        // Booster.Referee = &Referee;
        Booster.Init();
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object);

    #endif
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

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下失能
    {
        //遥控器离线或下方失能
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

        return;
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)  //左中正常随动
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
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP)  //左上
    {

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

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下失能
    {
        //遥控器离线或下方失能
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) //左中正常
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
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP)  //左上
    {
        //上方导航模式
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

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)  //左下失能
    {
        //遥控器离线或下方失能
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);

        return;
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) //左中
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
        //根据遥控器判断自家颜色, 上红下蓝
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            //MiniPC.Set_Self_Color(MiniPC_Self_Color_RED);
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
        {
            //MiniPC.Set_Self_Color(MiniPC_Self_Color_BLUE);
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
    #ifdef CHASSIS

        //底盘的控制策略
        Control_Chassis();
        //各个模块的分别解算
        Chassis.TIM_Calculate_PeriodElapsedCallback();

    #elif defined(GIMBAL)

        //云台发射机构的控制策略
        Control_Gimbal();
        Control_Booster();
        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster.TIM_Calculate_PeriodElapsedCallback();
	      //传输数据给上位机
	      MiniPC.TIM_Write_PeriodElapsedCallback();

    #endif   
}

/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        #ifdef CHASSIS

            chariot.Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

            for (auto& wheel : chariot.Chassis.Motor_Wheel) {
                wheel.TIM_Alive_PeriodElapsedCallback();
            }

        #elif defined(GIMBAL)
           
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();

            Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Pitch_LK6010.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

            Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
						
						MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
            
        #endif

        mod50 = 0;
    }    
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
