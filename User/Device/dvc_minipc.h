/**
 * @file dvc_minipc.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_MINIPC_H
#define DVC_MINIPC_H

/* Includes ------------------------------------------------------------------*/

#include "usbd_cdc_if.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 迷你主机状态
 *
 */
enum Enum_MiniPC_Status
{
    MiniPC_Status_DISABLE = 0,
    MiniPC_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum Enum_MiniPC_Data_Status : uint8_t
{
    MiniPC_Data_Status_DISABLE = 0,
    MiniPC_Data_Status_ENABLE,
};

/**
 * @brief 比赛阶段
 *
 */
enum Enum_MiniPC_Game_Stage : uint8_t
{
    MiniPC_Game_Stage_NOT_STARTED = 0,
    MiniPC_Game_Stage_READY,
    MiniPC_Game_Stage_SELF_TESTING,
    MiniPC_Game_Stage_5S_COUNTDOWN,
    MiniPC_Game_Stage_BATTLE,
    MiniPC_Game_Stage_SETTLEMENT,
};

/**
 * @brief 战车运动控制方式
 *
 */
enum Enum_MiniPC_Move_Control_Mode : uint8_t
{
    MiniPC_Move_Control_Mode_OMEGA = 0,
    MiniPC_Move_Control_Mode_ANGLE,
};

/**
 * @brief 战车运动控制方式
 *
 */
enum Enum_MiniPC_Self_Color : uint8_t
{
    MiniPC_Self_Color_RED = 0,
    MiniPC_Self_Color_BLUE,
};

/**
 * @brief 迷你主机源数据
 *
 */
struct Struct_MiniPC_USB_Data
{
    uint16_t Frame_Header;
    uint8_t Data[50];
} __attribute__((packed));

/**
 * @brief 迷你主机接收的规划数据,
 *
 */
struct Struct_MiniPC_Rx_Data
{
    uint8_t Reserved;
    float Chassis_Target_Velocity_X;
    float Chassis_Target_Velocity_Y;
    float Chassis_Target_Omega;
    float Chassis_Target_Angle;
    float Chassis_Now_Omega;
    float Chassis_Now_Angle;
    float Gimbal_Target_Yaw_Angle;
    float Gimbal_Target_Yaw_Omega;
    float Gimbal_Target_Pitch_Angle;
    float Gimbal_Target_Pitch_Omega;
    uint8_t Booster_Frequency;
    Enum_MiniPC_Move_Control_Mode Move_Control_Mode;
    uint8_t Frame_Rear;
} __attribute__((packed));

/**
 * @brief 迷你主机发送的反馈数据
 *
 */
struct Struct_MiniPC_Tx_Data
{
    Enum_MiniPC_Game_Stage Game_Stage;
    float Chassis_Now_Velocity_X;
    float Chassis_Now_Velocity_Y;
    float Chassis_Now_Omega;
    float Gimbal_Now_Yaw_Angle;
    float Gimbal_Now_Yaw_Omega;
    float Gimbal_Now_Pitch_Angle;
    float Gimbal_Now_Pitch_Omega;
    uint8_t Armor_Attacked_ID : 2;
    uint8_t Armor_Attacked_Ammo_Type_Enum : 1;
    uint8_t Self_Color : 1;
    uint8_t Outpost_Status_Enum : 1;
    uint8_t Outpost_Protect_Status_Enum : 1;
    uint8_t Reserved : 2;
    uint8_t Frame_Rear;
    uint8_t Carriage_Return;
} __attribute__((packed));

/**
 * @brief Specialized, 迷你主机类
 *
 */
class Class_MiniPC
{
public:
    void Init(uint16_t __Frame_Header = 0xfdfc, uint8_t __Frame_Rear = 0xfe);

    inline Enum_MiniPC_Status Get_MiniPC_Status();
    inline float Get_Chassis_Target_Velocity_X();
    inline float Get_Chassis_Target_Velocity_Y();
    inline float Get_Chassis_Target_Omega();
    inline float Get_Chassis_Target_Angle();
    inline float Get_Chassis_Now_Omega();
    inline float Get_Chassis_Now_Angle();
    inline float Get_Gimbal_Target_Yaw_Angle();
    inline float Get_Gimbal_Target_Yaw_Omega();
    inline float Get_Gimbal_Target_Pitch_Angle();
    inline float Get_Gimbal_Target_Pitch_Omega();
    inline float Get_Booster_Frequency();
    inline Enum_MiniPC_Move_Control_Mode Get_Move_Control_Mode();

    inline void Set_Game_Stage(Enum_MiniPC_Game_Stage __Game_Stage);
    inline void Set_Chassis_Now_Velocity_X(float __Chassis_Now_Velocity_X);
    inline void Set_Chassis_Now_Velocity_Y(float __Chassis_Now_Velocity_Y);
    inline void Set_Chassis_Now_Omega(float __Chassis_Now_Omega);
    inline void Set_Gimbal_Now_Yaw_Angle(float __Gimbal_Now_Yaw_Angle);
    inline void Set_Gimbal_Now_Yaw_Omega(float __Gimbal_Now_Yaw_Omega);
    inline void Set_Gimbal_Now_Pitch_Angle(float __Gimbal_Now_Pitch_Angle);
    inline void Set_Gimbal_Now_Pitch_Omega(float __Gimbal_Now_Pitch_Omega);
    inline void Set_Armor_Attacked_ID(uint8_t __Armor_Attacked_ID);
    inline void Set_Armor_Attacked_Ammo_Status(Enum_MiniPC_Data_Status __Armor_Attacked_Ammo_Status);
    inline void Set_Self_Color(Enum_MiniPC_Self_Color __Self_Color);
    inline void Set_Outpost_Status(Enum_MiniPC_Data_Status __Outpost_Status);
    inline void Set_Outpost_Protect_Status(Enum_MiniPC_Data_Status __Outpost_Protect_Status);

    void USB_RxCpltCallback(uint8_t *Rx_Data);
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void TIM_Write_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的USB
    Struct_USB_Manage_Object *USB_Manage_Object;
    //数据包头标
    uint16_t Frame_Header;
    //数据包尾标
    uint8_t Frame_Rear;

    //常量

    //内部变量

    //当前时刻的迷你主机接收flag
    uint32_t Flag = 0;
    //前一时刻的迷你主机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    //迷你主机对外接口信息
    Struct_MiniPC_Rx_Data Data_NUC_To_MCU;

    //写变量

    //迷你主机对外接口信息
    Struct_MiniPC_Tx_Data Data_MCU_To_NUC;

    //读写变量

    //内部函数

    void Data_Process();
    void Output();
};
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取迷你主机状态
 *
 * @return Enum_MiniPC_Status 迷你主机状态
 */
Enum_MiniPC_Status Class_MiniPC::Get_MiniPC_Status()
{
    return (MiniPC_Status);
}

/**
 * @brief 获取底盘目标速度x
 *
 * @return float 底盘目标速度x
 */
float Class_MiniPC::Get_Chassis_Target_Velocity_X()
{
    return (Data_NUC_To_MCU.Chassis_Target_Velocity_X);
}

/**
 * @brief 获取底盘目标速度y
 *
 * @return float 底盘目标速度y
 */
float Class_MiniPC::Get_Chassis_Target_Velocity_Y()
{
    return (Data_NUC_To_MCU.Chassis_Target_Velocity_Y);
}

/**
 * @brief 获取底盘目标速度omega
 *
 * @return float 获取底盘目标速度omega
 */
float Class_MiniPC::Get_Chassis_Target_Omega()
{
    return (Data_NUC_To_MCU.Chassis_Target_Omega);
}

/**
 * @brief 获取底盘目标角度
 *
 * @return float 获取底盘目标角度
 */
float Class_MiniPC::Get_Chassis_Target_Angle()
{
    return (Data_NUC_To_MCU.Chassis_Target_Angle);
}

/**
 * @brief 获取底盘当前速度omega
 *
 * @return float 获取底盘当前速度omega
 */
float Class_MiniPC::Get_Chassis_Now_Omega()
{
    return (Data_NUC_To_MCU.Chassis_Now_Omega);
}

/**
 * @brief 获取底盘当前角度
 *
 * @return float 获取底盘当前角度
 */
float Class_MiniPC::Get_Chassis_Now_Angle()
{
    return (Data_NUC_To_MCU.Chassis_Now_Angle);
}

/**
 * @brief 获取云台目标角度yaw
 *
 * @return float 云台目标角度yaw
 */
float Class_MiniPC::Get_Gimbal_Target_Yaw_Angle()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Yaw_Angle);
}

/**
 * @brief 获取云台目标速度yaw
 *
 * @return float 获取云台目标速度yaw
 */
float Class_MiniPC::Get_Gimbal_Target_Yaw_Omega()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Yaw_Omega);
}

/**
 * @brief 获取云台目标pitch
 *
 * @return float 云台目标pitch
 */
float Class_MiniPC::Get_Gimbal_Target_Pitch_Angle()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Pitch_Angle);
}

/**
 * @brief 获取云台目标速度pitch
 *
 * @return float 云台目标速度pitch
 */
float Class_MiniPC::Get_Gimbal_Target_Pitch_Omega()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Pitch_Omega);
}

/**
 * @brief 获取射频
 *
 * @return float 射频
 */
float Class_MiniPC::Get_Booster_Frequency()
{
    return (Data_NUC_To_MCU.Booster_Frequency);
}

/**
 * @brief 获取移动控制模式
 *
 * @return Enum_MiniPC_Move_Control_Mode 移动控制模式
 */
Enum_MiniPC_Move_Control_Mode Class_MiniPC::Get_Move_Control_Mode()
{
    return (Data_NUC_To_MCU.Move_Control_Mode);
}

/**
 * @brief
 *
 * @param __Game_Stage
 */
void Class_MiniPC::Set_Game_Stage(Enum_MiniPC_Game_Stage __Game_Stage)
{
    Data_MCU_To_NUC.Game_Stage = __Game_Stage;
}

/**
 * @brief 设定底盘逆解速度x
 *
 * @param __Chassis_Now_Velocity_X 底盘逆解速度x
 */
void Class_MiniPC::Set_Chassis_Now_Velocity_X(float __Chassis_Now_Velocity_X)
{
    Data_MCU_To_NUC.Chassis_Now_Velocity_X = __Chassis_Now_Velocity_X;
}

/**
 * @brief 设定底盘逆解速度y
 *
 * @param __Chassis_Now_Velocity_Y 底盘逆解速度y
 */
void Class_MiniPC::Set_Chassis_Now_Velocity_Y(float __Chassis_Now_Velocity_Y)
{
    Data_MCU_To_NUC.Chassis_Now_Velocity_Y = __Chassis_Now_Velocity_Y;
}

/**
 * @brief 设定底盘逆解速度omega
 *
 * @param __Chassis_Now_Omega 底盘逆解速度omega
 */
void Class_MiniPC::Set_Chassis_Now_Omega(float __Chassis_Now_Omega)
{
    Data_MCU_To_NUC.Chassis_Now_Omega = __Chassis_Now_Omega;
}

/**
 * @brief 设定云台当前角度yaw
 *
 * @param __Gimbal_Now_Yaw_Angle 云台当前角度yaw
 */
void Class_MiniPC::Set_Gimbal_Now_Yaw_Angle(float __Gimbal_Now_Yaw_Angle)
{
    Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle = __Gimbal_Now_Yaw_Angle;
}

/**
 * @brief 设定云台当前角速度yaw
 *
 * @param __Gimbal_Now_Yaw_Angle 云台当前角速度yaw
 */
void Class_MiniPC::Set_Gimbal_Now_Yaw_Omega(float __Gimbal_Now_Yaw_Omega)
{
    Data_MCU_To_NUC.Gimbal_Now_Yaw_Omega = __Gimbal_Now_Yaw_Omega;
}

/**
 * @brief 设定云台当前角度pitch
 *
 * @param __Gimbal_Now_Pitch_Angle 云台当前角度pitch
 */
void Class_MiniPC::Set_Gimbal_Now_Pitch_Angle(float __Gimbal_Now_Pitch_Angle)
{
    Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle = __Gimbal_Now_Pitch_Angle;
}

/**
 * @brief 设定云台当前角速度pitch
 *
 * @param __Gimbal_Now_Pitch_Angle 云台当前角速度pitch
 */
void Class_MiniPC::Set_Gimbal_Now_Pitch_Omega(float __Gimbal_Now_Pitch_Omega)
{
    Data_MCU_To_NUC.Gimbal_Now_Pitch_Omega = __Gimbal_Now_Pitch_Omega;
}

/**
 * @brief 设定装甲板受击ID
 *
 * @param __Armor_Attacked 装甲板受击ID
 */
void Class_MiniPC::Set_Armor_Attacked_ID(uint8_t __Armor_Attacked_ID)
{
    Data_MCU_To_NUC.Armor_Attacked_ID = __Armor_Attacked_ID;
}

/**
 * @brief 设定装甲板受击子弹类型
 *
 * @param __Armor_Attacked_Ammo_Type 装甲板受击子弹类型
 */
void Class_MiniPC::Set_Armor_Attacked_Ammo_Status(Enum_MiniPC_Data_Status __Armor_Attacked_Ammo_Status)
{
    Data_MCU_To_NUC.Armor_Attacked_Ammo_Type_Enum = __Armor_Attacked_Ammo_Status;
}

/**
 * @brief 设定己方颜色
 *
 * @param __Self_Color 己方颜色
 */
void Class_MiniPC::Set_Self_Color(Enum_MiniPC_Self_Color __Self_Color)
{
    Data_MCU_To_NUC.Self_Color = __Self_Color;
}

/**
 * @brief 设定前哨站状态
 *
 * @param __Outpost_Status 前哨站状态
 */
void Class_MiniPC::Set_Outpost_Status(Enum_MiniPC_Data_Status __Outpost_Status)
{
    Data_MCU_To_NUC.Outpost_Status_Enum = __Outpost_Status;
}

/**
 * @brief 设定前哨站保护状态
 *
 * @param __Outpost_Protect_Status 前哨站保护状态
 */
void Class_MiniPC::Set_Outpost_Protect_Status(Enum_MiniPC_Data_Status __Outpost_Protect_Status)
{
    Data_MCU_To_NUC.Outpost_Protect_Status_Enum = __Outpost_Protect_Status;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
