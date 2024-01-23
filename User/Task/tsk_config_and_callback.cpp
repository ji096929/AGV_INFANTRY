/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

bool init_finished = 0;

Class_Chariot chariot;

Class_DM_Motor_J4310 motor;
Class_BoardA_MPU boarda_mpu;
Class_Serialplot serialplot;
static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    // 电机调PID
    // "pa",
    // "ia",
    // "da",
    // "po",
    // "io",
    // "do",
    // "pt",
    // "it",
    // "dt",

    // 摩擦轮标定
    // "po",
    // "io",
    // "do",
    // "omega",

    // 功率控制调参
    // "ps",
    // "is",
    // "ds",
    // "pw",
    // "iw",
    // "dw",

    // 陀螺仪调参
    "kp",
    "ki",
    "int",
};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0xf1):
    {
        motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x201):
    {
        chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x205):
    {
        chariot.Chassis.Motor_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x207):
    {
        chariot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x202):
    {
        chariot.Booster.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Booster.Motor_Friction_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Booster.Motor_Friction_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x205):
    {
        chariot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        chariot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}

/**
 * @brief SPI5回调函数
 *
 * @param Tx_Buffer SPI5发送的消息
 * @param Rx_Buffer SPI5接收的消息
 * @param Length 长度
 */
void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{
    if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
    {
        boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
    }
}

/**
 * @brief UART1遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void DR16_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.UART_RxCpltCallback(Buffer);
}

/**
 * @brief UART2串口绘图回调函数
 *
 * @param Buffer UART2收到的消息
 * @param Length 长度
 */
void Serialplot_UART2_Callback(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
    // 电机调PID
    //  case(0):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(1):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(2):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(3):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(4):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(5):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(6):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Torque.Set_K_P(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(7):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Torque.Set_K_I(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(8):
    //  {
    //      chariot.Gimbal.Motor_Yaw.PID_Torque.Set_K_D(serialplot.Get_Variable_Value());
    //  }
    //  break;

    // 摩擦轮标定
    //  case(0):
    //  {
    //      chariot.Booster.Motor_Friction_Left.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
    //      chariot.Booster.Motor_Friction_Right.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(1):
    //  {
    //      chariot.Booster.Motor_Friction_Left.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
    //      chariot.Booster.Motor_Friction_Right.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(2):
    //  {
    //      chariot.Booster.Motor_Friction_Left.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
    //      chariot.Booster.Motor_Friction_Right.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(3):
    //  {
    //      chariot.Booster.Set_Friction_Omega(serialplot.Get_Variable_Value());
    //  }
    //  break;

    // 功率控制调参
    //  case(0):
    //  {
    //      chariot.Chassis.PID_Power_Limit_Steer.Set_K_P(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(1):
    //  {
    //      chariot.Chassis.PID_Power_Limit_Steer.Set_K_I(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(2):
    //  {
    //      chariot.Chassis.PID_Power_Limit_Steer.Set_K_D(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(3):
    //  {
    //      chariot.Chassis.PID_Power_Limit_Wheel.Set_K_P(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(4):
    //  {
    //      chariot.Chassis.PID_Power_Limit_Wheel.Set_K_I(serialplot.Get_Variable_Value());
    //  }
    //  break;
    //  case(5):
    //  {
    //      chariot.Chassis.PID_Power_Limit_Wheel.Set_K_D(serialplot.Get_Variable_Value());
    //  }
    //  break;

    // 陀螺仪调参
    case (0):
    {
        boarda_mpu.PID_Mahony_X.Set_K_P(serialplot.Get_Variable_Value());
        boarda_mpu.PID_Mahony_Y.Set_K_P(serialplot.Get_Variable_Value());
        boarda_mpu.PID_Mahony_Z.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
    case (1):
    {
        boarda_mpu.PID_Mahony_X.Set_K_I(serialplot.Get_Variable_Value());
        boarda_mpu.PID_Mahony_Y.Set_K_I(serialplot.Get_Variable_Value());
        boarda_mpu.PID_Mahony_Z.Set_K_I(serialplot.Get_Variable_Value());
    }
    break;
    case (2):
    {
        boarda_mpu.PID_Mahony_X.Set_Integral_Error(serialplot.Get_Variable_Value());
        boarda_mpu.PID_Mahony_Y.Set_Integral_Error(serialplot.Get_Variable_Value());
        boarda_mpu.PID_Mahony_Z.Set_Integral_Error(serialplot.Get_Variable_Value());
    }
    break;
    }
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer);
}

/**
 * @brief UART7云台AHRS回调函数
 *
 * @param Buffer UART7收到的消息
 * @param Length 长度
 */
void AHRS_UART7_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Gimbal.WIT.UART_RxCpltCallback(Buffer);
}

/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer UART7收到的消息
 *
 * @param Length 长度
 */
void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    chariot.MiniPC.USB_RxCpltCallback(Buffer);
}

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
    // 单给收发SPI消息开的定时器
    boarda_mpu.TIM100us_Send_PeriodElapsedCallback();
}

/**
 * @brief TIM5任务回调函数
 *
 */
void Task1ms_TIM5_Callback()
{
    // 判断遥控器, WIT姿态传感器, 裁判系统是否断开连接
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        chariot.Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        chariot.DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        chariot.Gimbal.WIT.TIM1msMod50_Alive_PeriodElapsedCallback();
        chariot.MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

        motor.TIM_Alive_PeriodElapsedCallback();
    }

    // 交互层回调函数

    chariot.TIM_Control_Callback();

    // 设备层回调函数

    // MPU的滤波器
    boarda_mpu.TIM_Calculate_PeriodElapsedCallback();

    // 电机调PID串口绘图
    //  float angle_target, angle_now, omega_target, omega_now, torque_target, torque_now;
    //  angle_now = chariot.Chassis.Motor_Steer[0].Get_Now_Angle();
    //  angle_target = chariot.Chassis.Motor_Steer[0].Get_Target_Angle();
    //  omega_now = chariot.Chassis.Motor_Steer[0].Get_Now_Omega();
    //  omega_target = chariot.Chassis.Motor_Steer[0].Get_Target_Omega();
    //  torque_now = chariot.Chassis.Motor_Steer[0].Get_Now_Torque();
    //  torque_target = chariot.Chassis.Motor_Steer[0].Get_Target_Torque();
    //  serialplot.Set_Data(6, &angle_now, &angle_target, &omega_now, &omega_target, &torque_now, &torque_target);

    // 摩擦轮标定串口绘图
    //  float omega_target, omega_now, torque_target, torque_now;
    //  omega_now = chariot.Booster.Motor_Friction_Left.Get_Now_Omega();
    //  omega_target = chariot.Booster.Motor_Friction_Left.Get_Target_Omega();
    //  torque_now = chariot.Booster.Motor_Friction_Left.Get_Now_Torque();
    //  torque_target = chariot.Booster.Motor_Friction_Left.Get_Target_Torque();
    //  serialplot.Set_Data(4, &omega_now, &omega_target, &torque_now, &torque_target);

    // 陀螺仪获取串口绘图
    //  float omega_x, omega_y, omega_z;
    //  omega_x = chariot.Gimbal.AHRS_WIT.Get_Omega_X();
    //  omega_y = chariot.Gimbal.AHRS_WIT.Get_Omega_Y();
    //  omega_z = chariot.Gimbal.AHRS_WIT.Get_Omega_Z();
    //  serialplot.Set_Data(3, &omega_x, &omega_y, &omega_z);

    // 功率控制串口绘图
    //  float sampler_power, referee_power;
    //  float target_steer_power, now_steer_power;
    //  float target_wheel_power, now_wheel_power;
    //  float sampler_out, out_steer_power, out_wheel_power;
    //  sampler_power = chariot.Chassis.Get_Now_Power();
    //  referee_power = chariot.Referee.Get_Chassis_Power();
    //  target_steer_power = chariot.Chassis.Get_Target_Steer_Power();
    //  now_steer_power = chariot.Chassis.Get_Now_Steer_Power();
    //  target_wheel_power = chariot.Chassis.Get_Target_Wheel_Power();
    //  now_wheel_power = chariot.Chassis.Get_Now_Wheel_Power();
    //  sampler_out = chariot.Chassis.Sampler.Get_Value();
    //  out_steer_power = chariot.Chassis.PID_Power_Limit_Steer.Get_Out();
    //  out_wheel_power = chariot.Chassis.PID_Power_Limit_Wheel.Get_Out();
    //  serialplot.Set_Data(9, &sampler_power, &referee_power, &target_steer_power, &now_steer_power, &target_wheel_power, &now_wheel_power, &sampler_out, &out_steer_power, &out_wheel_power);

    // 底盘信息串口绘图
    //  float target_vx, target_vy, target_omega, real_vx, real_vy, real_omega;
    //  target_vx = chariot.Chassis.Get_Target_Velocity_X();
    //  target_vy = chariot.Chassis.Get_Target_Velocity_Y();
    //  target_omega = chariot.Chassis.Get_Target_Omega();
    //  real_vx = chariot.Chassis.Slope_Velocity_X.Get_Out();
    //  real_vy = chariot.Chassis.Slope_Velocity_Y.Get_Out();
    //  real_omega = chariot.Chassis.Slope_Omega.Get_Out();
    //  serialplot.Set_Data(6, &target_vx, &target_vy, &target_omega, &real_vx, &real_vy, &real_omega);

    // 发射机构信息串口绘图
    //  float heat, heat_max, heat_cd, friction_torque, friction_omega;
    //  heat = chariot.Booster.FSM_Heat_Detect.Heat;
    //  heat_max = chariot.Booster.Referee->Get_Booster_17mm_1_Heat_Max();
    //  heat_cd = chariot.Booster.Referee->Get_Booster_17mm_1_Heat_CD();
    //  friction_torque = chariot.Booster.Motor_Friction_Right.Get_Now_Torque();
    //  friction_omega = chariot.Booster.Motor_Friction_Right.Get_Now_Omega();
    //  serialplot.Set_Data(6, &heat, &heat_max, &heat_cd, &heat_cd, &friction_torque, &friction_omega);

    // 陀螺仪基本数据测试
    //  float tempax = boarda_mpu.Get_Accelerate_X();
    //  float tempay = boarda_mpu.Get_Accelerate_Y();
    //  float tempaz = boarda_mpu.Get_Accelerate_Z();
    //  float temptemp = boarda_mpu.Get_Temperature();
    //  float tempox = boarda_mpu.Get_Omega_X();
    //  float tempoy = boarda_mpu.Get_Omega_Y();
    //  float tempoz = boarda_mpu.Get_Omega_Z();
    //  float tempmx = boarda_mpu.Get_Magnet_X();
    //  float tempmy = boarda_mpu.Get_Magnet_Y();
    //  float tempmz = boarda_mpu.Get_Magnet_Z();
    //  serialplot.Set_Data(11, &tempax, &tempay, &tempaz, &temptemp, &tempox, &tempoy, &tempoz, &tempmx, &tempmy, &tempmz);

    // 陀螺仪解算测试
    //  float tempq0 = boarda_mpu.Get_Quaternion_0();
    //  float tempq1 = boarda_mpu.Get_Quaternion_1();
    //  float tempq2 = boarda_mpu.Get_Quaternion_2();
    //  float tempq3 = boarda_mpu.Get_Quaternion_3();
    //  float tempyaw = boarda_mpu.Get_Angle_Yaw();
    //  float temppitch = boarda_mpu.Get_Angle_Pitch();
    //  float temproll = boarda_mpu.Get_Angle_Roll();
    //  serialplot.Set_Data(7, &tempq0, &tempq1, &tempq2, &tempq3, &tempyaw, &temppitch, &temproll);

    // 达妙电机测试
    float tempangle = motor.Get_Now_Angle();
    float tempvelocity = motor.Get_Now_Omega();
    float temptorque = motor.Get_Now_Torque();
    serialplot.Set_Data(3, &tempangle, &tempvelocity, &temptorque);

    serialplot.TIM_Write_PeriodElapsedCallback();

    motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    static int flag = 0;
    flag++;
    if (flag == 5000)
    {
        motor.Set_Target_Angle(PI);
    }
    else if (flag == 10000)
    {
        flag = 0;
        motor.Set_Target_Angle(-PI);
    }
    motor.Set_Target_Omega(10.0f * PI);
    motor.TIM_Process_PeriodElapsedCallback();

    // 驱动层回调函数

    TIM_CAN_PeriodElapsedCallback();
    TIM_UART_PeriodElapsedCallback();

    static int mod5 = 0;
    mod5++;
    if (mod5 == 5)
    {
        mod5 = 0;

        TIM_USB_PeriodElapsedCallback();
    }
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    // 驱动层初始化

    BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);

    ADC_Init(&hadc1, 1);

    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);

    SPI_Init(&hspi5, Device_SPI5_Callback);

    UART_Init(&huart1, DR16_UART1_Callback, 18);
    UART_Init(&huart2, Serialplot_UART2_Callback, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    UART_Init(&huart6, Referee_UART6_Callback, 128);
    UART_Init(&huart7, AHRS_UART7_Callback, 11);

    USB_Init(MiniPC_USB_Callback, 64);

    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    // 设备层初始化

    boarda_mpu.Init();
    serialplot.Init(&huart2, 6, (char **)Variable_Assignment_List);
    motor.Init(&hcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA);

    // 交互层初始化

    chariot.Init();

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);

    init_finished = 1;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
