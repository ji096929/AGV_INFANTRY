/**
 * @file dvc_motor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆电机配置与操作
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_DJIMOTOR_H
#define DVC_DJIMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"
#include "alg_power_limit.h"

/* Exported macros -----------------------------------------------------------*/
//弧度转化
#define RADPS_TO_RPM(x) ((x) * (60.0f / (2.0f * PI)))
#define RAD_TO_ANGEL(x) ((x) * (180.0f / PI))
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆状态
 *
 */
enum Enum_DJI_Motor_Status
{
    DJI_Motor_Status_DISABLE = 0,
    DJI_Motor_Status_ENABLE,
};

/**
 * @brief 大疆电机的ID枚举类型
 *
 */
enum Enum_DJI_Motor_ID
{
    DJI_Motor_ID_UNDEFINED = 0,
    DJI_Motor_ID_0x201,
    DJI_Motor_ID_0x202,
    DJI_Motor_ID_0x203,
    DJI_Motor_ID_0x204,
    DJI_Motor_ID_0x205,
    DJI_Motor_ID_0x206,
    DJI_Motor_ID_0x207,
    DJI_Motor_ID_0x208,
    DJI_Motor_ID_0x209,
    DJI_Motor_ID_0x20A,
    DJI_Motor_ID_0x20B,
};

/**
 * @brief 大疆电机控制方式
 *
 */
enum Enum_DJI_Motor_Control_Method
{
    DJI_Motor_Control_Method_OPENLOOP = 0,
    DJI_Motor_Control_Method_TORQUE,
    DJI_Motor_Control_Method_OMEGA,
    DJI_Motor_Control_Method_ANGLE,
    DJI_Motor_Control_Method_IMU_OMEGA,
    DJI_Motor_Control_Method_IMU_ANGLE,
};

/**
 * @brief 大疆电机源数据
 *
 */
struct Struct_DJI_Motor_CAN_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Torque_Reverse;
    int8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 大疆电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_DJI_Motor_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief GM6020无刷电机, 单片机控制输出电压
 *
 */
class Class_DJI_Motor_GM6020
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    // PID扭矩环控制
    Class_PID PID_Torque;

    void Init(CAN_HandleTypeDef *__hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __Control_Method = DJI_Motor_Control_Method_ANGLE, int32_t __Encoder_Offset = 0, float __Omega_Max = 320.0f * RPM_TO_RADPS);

    inline uint16_t Get_Output_Max();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline uint8_t Get_Now_Temperature();
    inline Enum_DJI_Motor_Control_Method Get_Control_Method();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Out();

    inline void Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_DJI_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //编码器偏移
    uint32_t Encoder_Offset;
    //最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
    float Omega_Max;

    //常量

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
    //最大输出电压
    uint16_t Output_Max = 25000;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //电机状态
    Enum_DJI_Motor_Status DJI_Motor_Status = DJI_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_DJI_Motor_Data Data;

    //写变量

    //读写变量

    //电机控制方式
    Enum_DJI_Motor_Control_Method DJI_Motor_Control_Method = DJI_Motor_Control_Method_IMU_ANGLE;
    //目标的角度, 度数
    float Target_Angle = 0.0f;
    //目标的速度, rpm
    float Target_Omega = 0.0f;
    //目标的扭矩, 直接采用反馈值
    float Target_Torque = 0.0f;
    //输出量
    float Out = 0.0f;

    //内部函数

    void Data_Process();
    void Output();
};

/**
 * @brief C610无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
class Class_DJI_Motor_C610
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(CAN_HandleTypeDef *__hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __Control_Method = DJI_Motor_Control_Method_OMEGA, float __Gearbox_Rate = 36.0f, float __Torque_Max = 10000.0f);

    inline uint16_t Get_Output_Max();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline uint8_t Get_Now_Temperature();
    inline Enum_DJI_Motor_Control_Method Get_Control_Method();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Out();

    inline void Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_DJI_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //减速比, 默认带减速箱
    float Gearbox_Rate = 36.0f;
    //最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
    float Torque_Max;

    //常量

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
    //最大输出扭矩
    uint16_t Output_Max = 10000;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //电机状态
    Enum_DJI_Motor_Status DJI_Motor_Status = DJI_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_DJI_Motor_Data Data;

    //写变量

    //读写变量

    //电机控制方式
    Enum_DJI_Motor_Control_Method DJI_Motor_Control_Method = DJI_Motor_Control_Method_ANGLE;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的扭矩, 直接采用反馈值
    float Target_Torque = 0.0f;
    //输出量
    float Out = 0.0f;

    //内部函数

    void Data_Process();
    void Output();
};

/**
 * @brief C620无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
class Class_DJI_Motor_C620
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    
    //功率限制友元函数
    friend class Class_Power_Limit;

    void Init(CAN_HandleTypeDef *__hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __Control_Method = DJI_Motor_Control_Method_OMEGA, float __Gearbox_Rate = 13.933f, float __Torque_Max = 16384.0f);

    inline uint16_t Get_Output_Max();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline uint8_t Get_Now_Temperature();
    inline Enum_DJI_Motor_Control_Method Get_Control_Method();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Out();

    inline void Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_DJI_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //减速比, 默认带减速箱
    float Gearbox_Rate;
    //最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
    float Torque_Max;

    //常量

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
    //最大输出扭矩
    uint16_t Output_Max = 16384;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //电机状态
    Enum_DJI_Motor_Status DJI_Motor_Status = DJI_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_DJI_Motor_Data Data;

    //写变量

    //读写变量

    //电机控制方式
    Enum_DJI_Motor_Control_Method DJI_Motor_Control_Method = DJI_Motor_Control_Method_ANGLE;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的扭矩, 直接采用反馈值
    float Target_Torque = 0.0f;
    //输出量
    float Out = 0.0f;

    //内部函数

    void Data_Process();
    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取最大输出电压
 *
 * @return uint16_t 最大输出电压
 */
uint16_t Class_DJI_Motor_GM6020::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_DJI_Motor_Status 电机状态
 */
Enum_DJI_Motor_Status Class_DJI_Motor_GM6020::Get_DJI_Motor_Status()
{
    return (DJI_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DJI_Motor_GM6020::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DJI_Motor_GM6020::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_GM6020::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_DJI_Motor_GM6020::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DJI_Motor_Control_Method 电机控制方式
 */
Enum_DJI_Motor_Control_Method Class_DJI_Motor_GM6020::Get_Control_Method()
{
    return (DJI_Motor_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DJI_Motor_GM6020::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DJI_Motor_GM6020::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_GM6020::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_DJI_Motor_GM6020::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DJI_Motor_Control_Method 电机控制方式
 */
void Class_DJI_Motor_GM6020::Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method)
{
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DJI_Motor_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DJI_Motor_GM6020::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_DJI_Motor_GM6020::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_DJI_Motor_GM6020::Set_Out(float __Out)
{
    Out = __Out;
}

/**
 * @brief 获取最大输出电流
 *
 * @return uint16_t 最大输出电流
 */
uint16_t Class_DJI_Motor_C610::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_DJI_Motor_Status 电机状态
 */
Enum_DJI_Motor_Status Class_DJI_Motor_C610::Get_DJI_Motor_Status()
{
    return (DJI_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DJI_Motor_C610::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DJI_Motor_C610::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C610::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_DJI_Motor_C610::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DJI_Motor_Control_Method 电机控制方式
 */
Enum_DJI_Motor_Control_Method Class_DJI_Motor_C610::Get_Control_Method()
{
    return (DJI_Motor_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DJI_Motor_C610::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DJI_Motor_C610::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C610::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_DJI_Motor_C610::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DJI_Motor_Control_Method 电机控制方式
 */
void Class_DJI_Motor_C610::Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method)
{
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DJI_Motor_C610::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DJI_Motor_C610::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_DJI_Motor_C610::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_DJI_Motor_C610::Set_Out(float __Out)
{
    Out = __Out;
}

/**
 * @brief 获取最大输出电流
 *
 * @return uint16_t 最大输出电流
 */
uint16_t Class_DJI_Motor_C620::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_DJI_Motor_Status 电机状态
 */
Enum_DJI_Motor_Status Class_DJI_Motor_C620::Get_DJI_Motor_Status()
{
    return (DJI_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DJI_Motor_C620::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DJI_Motor_C620::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C620::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_DJI_Motor_C620::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DJI_Motor_Control_Method 电机控制方式
 */
Enum_DJI_Motor_Control_Method Class_DJI_Motor_C620::Get_Control_Method()
{
    return (DJI_Motor_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DJI_Motor_C620::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DJI_Motor_C620::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C620::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_DJI_Motor_C620::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DJI_Motor_Control_Method 电机控制方式
 */
void Class_DJI_Motor_C620::Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method)
{
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DJI_Motor_C620::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DJI_Motor_C620::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_DJI_Motor_C620::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_DJI_Motor_C620::Set_Out(float __Out)
{
    Out = __Out;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
