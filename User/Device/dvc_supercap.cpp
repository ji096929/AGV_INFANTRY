/**
 * @file dvc_supercap.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 超级电容
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_supercap.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化超级电容通信, 切记__CAN_ID避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 收数据绑定的CAN ID
 * @param __Limit_Power_Max 最大限制功率, 0表示不限制
 */
void Class_Supercap::Init(CAN_HandleTypeDef *hcan, uint16_t __CAN_ID, float __Limit_Power_Max)
{
    if(hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    Limit_Power_Max = __Limit_Power_Max;
    CAN_Tx_Data = CAN_Supercap_Tx_Data;
}

/**
 * @brief 
 * 
 */
void Class_Supercap::Data_Process()
{
    //数据处理过程
    Struct_Supercap_CAN_Data *tmp_buffer = (Struct_Supercap_CAN_Data *)CAN_Manage_Object->Rx_Buffer.Data;
    int16_t tmp_stored_energy, tmp_now_power;

    Math_Endian_Reverse_16((void *)&tmp_buffer->Stored_Energy_Reverse, (void *)&tmp_stored_energy);
    Supercap_Data.Stored_Energy = tmp_stored_energy;
    Math_Endian_Reverse_16((void *)&tmp_buffer->Now_Power_Reverse, (void *)&tmp_now_power);
    Supercap_Data.Now_Power = tmp_now_power / 100.0f;
}

/**
 * @brief 
 * 
 */
void Class_Supercap::Output()
{

}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Supercap::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测超级电容是否存活
 * 
 */
void Class_Supercap::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过超级电容数据
    if (Flag == Pre_Flag)
    {
        //超级电容断开连接
        Supercap_Status = Supercap_Status_DISABLE;
    }
    else
    {
        //超级电容保持连接
        Supercap_Status = Supercap_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
