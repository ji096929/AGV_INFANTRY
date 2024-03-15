/**
 * @file dvc_minipc.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright ustc-robowalker (c) 2023
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object* __USB_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
	  USB_Manage_Object = __USB_Manage_Object;
    Frame_Header = __frame_header;
    Frame_Rear = __frame_rear;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
    memcpy(&Data_NUC_To_MCU, ((Struct_MiniPC_USB_Data *)USB_Manage_Object->Rx_Buffer)->Data, sizeof(Struct_MiniPC_Rx_Data));
    memset(USB_Manage_Object->Rx_Buffer, 0, USB_Manage_Object->Rx_Buffer_Length);
}

/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */
void Class_MiniPC::Output()
{
    uint8_t *tmp_buffer = USB_Manage_Object->Tx_Buffer;
	  uint16_t tx_len = USB_Manage_Object->Tx_Buffer_Length;
    
    memset(tmp_buffer, 0, tx_len);
    
    tmp_buffer[0] = Frame_Header; //帧头
    memcpy(&tmp_buffer[1], &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
    tmp_buffer[tx_len-1] = Frame_Rear; //帧尾
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
    Output();
}

/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
    //滑动窗口, 判断迷你主机是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        //迷你主机断开连接
        MiniPC_Status =  MiniPC_Status_DISABLE;
    }
    else
    {
        //迷你主机保持连接
        MiniPC_Status =  MiniPC_Status_ENABLE ;
    }

    Pre_Flag = Flag;
}

/************************ copyright(c) ustc-robowalker **************************/
