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
#include "drv_math.h"
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
        
        Chassis.Set_Velocity_X_Max(10.0f);
        Chassis.Set_Velocity_Y_Max(10.0f);

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
 * @brief can回调函数处理云台发来的数据
 *
 */

void Class_Chariot::CAN_Chassis_Control_RxCpltCallback()
{
    //底盘和云台夹角（弧度制）
    float derta_angle;
    //float映射到int16之后的速度
    uint16_t tmp_velocity_x ,tmp_velocity_y;
    //底盘控制类型
    Enum_Chassis_Control_Type tmp_chassis_control_type;
    //底盘角速度
    float target_omega;
    //云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    
    memcpy(&tmp_velocity_x,&CAN_Manage_Object->Rx_Buffer.Data[0],sizeof(int16_t));
    memcpy(&tmp_velocity_y,&CAN_Manage_Object->Rx_Buffer.Data[2],sizeof(int16_t));
    memcpy(&tmp_chassis_control_type,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(uint8_t)); 
    // Math_Endian_Reverse_16((void *)CAN_Manage_Object->Rx_Buffer.Data[0], (void *)&tmp_velocity_x);
    // Math_Endian_Reverse_16((void *)CAN_Manage_Object->Rx_Buffer.Data[2], (void *)&tmp_velocity_y);
    
    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x,0,0x7FFF,-1 * Chassis.Get_Velocity_X_Max()* Chassis.Get_Velocity_X_Max(),Chassis.Get_Velocity_X_Max()*Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y,0,0x7FFF,-1 * Chassis.Get_Velocity_Y_Max()* Chassis.Get_Velocity_Y_Max(),Chassis.Get_Velocity_Y_Max()*Chassis.Get_Velocity_Y_Max());

//    //获取云台坐标系和底盘坐标系的夹角（弧度制）
//    Chassis_Angle = Chassis.Motor_Yaw.Get_Now_Angle();
//    derta_angle = Chassis_Angle - Reference_Angle;  

//    //云台坐标系的目标速度转为底盘坐标系的目标速度
//    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
//    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));

    //设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(tmp_chassis_control_type);
    
    //底盘控制方案
    if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        target_omega = Chassis.Get_Spin_Omega();
    }
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
        Chassis.PID_Chassis_Fllow.Set_Target(Reference_Angle);
        Chassis.PID_Chassis_Fllow.Set_Now(Chassis.Motor_Yaw.Get_Now_Angle());
        Chassis.PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        target_omega = Chassis.PID_Chassis_Fllow.Get_Out();
    }
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        target_omega = 0;
    }
    else
    {
        target_omega = 0;
    }
    
    //设定底盘目标速度
    Chassis.Set_Target_Velocity_X(gimbal_velocity_x);
    Chassis.Set_Target_Velocity_Y(gimbal_velocity_y);
    Chassis.Set_Target_Omega(target_omega);
}

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
void Class_Chariot::CAN_Gimbal_RxCpltCallback()
{

}

/**
 * @brief 底盘控制逻辑
 *
 */  		
    //遥控器摇杆值
		float dr16_l_x, dr16_l_y;    
		//云台坐标系速度目标值 float
		float gimbal_velocity_x = 0, gimbal_velocity_y = 0;    
		//映射之后的目标速度 int16_t
		uint16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type; 
void Class_Chariot::Control_Chassis()
{

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下失能
    {
        //遥控器离线或左下方失能
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        return;
    }
    else 
    {
        //遥控器操作逻辑

        //排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

        //设定矩形到圆形映射进行控制
        gimbal_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max() * Chassis.Get_Velocity_X_Max();
        gimbal_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max() * Chassis.Get_Velocity_Y_Max();

        //键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)  //左中 随动模式
        {
            //底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);   
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP)  //左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
        }

        if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED)  //x轴
        {
            gimbal_velocity_x -= Chassis.Get_Velocity_X_Max();
        }
        if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
        {
            gimbal_velocity_x += Chassis.Get_Velocity_X_Max();
        }
        if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED)  //y轴
        {
            gimbal_velocity_y += Chassis.Get_Velocity_Y_Max();
        }
        if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
        {
            gimbal_velocity_y -= Chassis.Get_Velocity_Y_Max();
        }
    }
    
    //设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(gimbal_velocity_x,-1 * Chassis.Get_Velocity_X_Max() * Chassis.Get_Velocity_X_Max(),Chassis.Get_Velocity_X_Max() * Chassis.Get_Velocity_X_Max(),0,0x7FFF);
    memcpy(CAN2_Chassis_Tx_Data, &tmp_chassis_velocity_x, sizeof(int16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(gimbal_velocity_y,-1 * Chassis.Get_Velocity_Y_Max() * Chassis.Get_Velocity_Y_Max(),Chassis.Get_Velocity_Y_Max() * Chassis.Get_Velocity_Y_Max(),0,0x7FFF);
    memcpy(CAN2_Chassis_Tx_Data + 2, &tmp_chassis_velocity_y, sizeof(int16_t));
    
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    memcpy(CAN2_Chassis_Tx_Data + 4,&chassis_control_type ,sizeof(Enum_Chassis_Control_Type));
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
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    #ifdef CHASSIS

        //底盘的控制策略
//        Control_Chassis();
        //各个模块的分别解算
        Chassis.TIM_Calculate_PeriodElapsedCallback();

    #elif defined(GIMBAL)

        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster.TIM_Calculate_PeriodElapsedCallback();
        //传输数据给上位机
        MiniPC.TIM_Write_PeriodElapsedCallback();

    #endif   
}

/**
 * @brief 控制回调函数
 *
 */
void Class_Chariot::TIM_Control_Callback()
{
    //底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
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

            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

            for (auto& wheel : Chassis.Motor_Wheel) {
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
