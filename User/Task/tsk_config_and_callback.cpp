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

//机器人控制对象
Class_Chariot chariot;



//串口裁判系统对象
Class_Serialplot serialplot;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
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
        case (0x204):
        {
            chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):  
        {
            
        }
        break;
        case (0x207):
        {
            
        }
        break;
    }
}

/**
 * @brief Chassis_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Chassis_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x202):  //留给yaw电机编码器回传
    {
        
    }
    break;
    case (0x203):  //留给上板通讯
    {
        
    }
    break;
    case (0x204):  //留给超级电容
    {
        
    }
    break;
    case (0x205):
    {
        
    }
    break;
    case (0x206):
    {
        
    }
    break;
	}
}

/**
 * @brief Gimbal_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Gimbal_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
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
        chariot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        
    }
    break;
	}
		
}

/**
 * @brief Gimbal_CAN1回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Gimbal_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x202):   //留给下板通讯
    {
        
    }
    break;
    case (0x203):
    {
        
    }
    break;
    case (0x204):
    {
        
    }
    break;
    case (0x205):
    {
       
    }
    break;
    case (0x206):
    {
        
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
//void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
//{
//    if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
//    {
//        boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
//    }
//}

/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{

}

/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.UART_RxCpltCallback(Buffer);
}


/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    
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
    //chariot.MiniPC.USB_RxCpltCallback(Buffer);
}

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
    #ifdef CHASSIS
        
        //暂无云台tim4任务

    #elif defined(GIMBAL)

        // 单给IMU消息开的定时器
        chariot.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();

    #endif
}

/**
 * @brief TIM5任务回调函数
 *
 */
void Task1ms_TIM5_Callback()
{
    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    
    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();

    /****************************** 交互层回调函数 1ms *****************************************/

    chariot.TIM_Control_Callback();

    /****************************** 驱动层回调函数 1ms *****************************************/ 
    
    //统一打包发送
    TIM_CAN_PeriodElapsedCallback();

    TIM_UART_PeriodElapsedCallback();

    static int mod5 = 0;
    mod5++;
    if (mod5 == 5)
    {
        //TIM_USB_PeriodElapsedCallback();
        mod5 = 0;
    }
    
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{  

    DWT_Init(168);

    /********************************** 驱动层初始化 **********************************/
	#ifdef CHASSIS

        //集中总线can1/can2
        CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
        CAN_Init(&hcan2, Chassis_Device_CAN2_Callback);

        //裁判系统
        UART_Init(&huart6, Referee_UART6_Callback, 128);

    #endif

    #ifdef GIMBAL

        //集中总线can1/can2
        CAN_Init(&hcan1, Gimbal_Device_CAN1_Callback);
        CAN_Init(&hcan2, Gimbal_Device_CAN2_Callback);

        //c板陀螺仪spi外设
        SPI_Init(&hspi1,Device_SPI1_Callback);

        //磁力计iic外设
        IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    
        
        //遥控器接收
        UART_Init(&huart3, DR16_UART3_Callback, 18);

    #endif

    //定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

     //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init();

    /********************************* 使能调度时钟 *********************************/

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
