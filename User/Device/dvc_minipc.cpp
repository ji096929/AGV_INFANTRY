/**
 * @file dvc_minipc.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化
 *
 * @param __Frame_Header 数据包头标
 * @param __Frame_Rear 数据包尾标
 */
void Class_MiniPC::Init(uint16_t __Frame_Header, uint8_t __Frame_Rear)
{
    USB_Manage_Object = &USBD_Manage_Object;
    Frame_Header = __Frame_Header;
    Frame_Rear = __Frame_Rear;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
    memcpy(&Data_NUC_To_MCU, ((Struct_MiniPC_USB_Data *)USB_Manage_Object->Rx_Buffer)->Data, sizeof(Struct_MiniPC_Rx_Data));
}

/**
 * @brief 迷你主机发送数据输出到USB发送缓冲区
 *
 */
void Class_MiniPC::Output()
{
    uint8_t *tmp_buffer = USB_Manage_Object->Tx_Buffer;
    
    memset(tmp_buffer, 0, USB_BUFFER_SIZE);

    memcpy(&tmp_buffer[0], &Frame_Header, sizeof(uint16_t));
    memcpy(&tmp_buffer[2], &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
    tmp_buffer[32] = 0xfe;
    tmp_buffer[33] = '\n';
}

/**
 * @brief TIM定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
    Output();
}

/**
 * @brief USB通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断迷你主机是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        //迷你主机断开连接
        MiniPC_Status = MiniPC_Status_DISABLE;
    }
    else
    {
        //迷你主机保持连接
        MiniPC_Status = MiniPC_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
