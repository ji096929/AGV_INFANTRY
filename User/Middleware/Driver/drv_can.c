/**
 * @file drv_can.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "ita_chariot.h"
#include "drv_can.h"
#include "main.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0xxf1_Tx_Data[8];
uint8_t CAN1_0xxf2_Tx_Data[8];
uint8_t CAN1_0xxf3_Tx_Data[8];
uint8_t CAN1_0xxf4_Tx_Data[8];
uint8_t CAN1_0xxf5_Tx_Data[8];
uint8_t CAN1_0xxf6_Tx_Data[8];
uint8_t CAN1_0xxf7_Tx_Data[8];
uint8_t CAN1_0xxf8_Tx_Data[8];

uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0xxf1_Tx_Data[8];
uint8_t CAN2_0xxf2_Tx_Data[8];
uint8_t CAN2_0xxf3_Tx_Data[8];
uint8_t CAN2_0xxf4_Tx_Data[8];
uint8_t CAN2_0xxf5_Tx_Data[8];
uint8_t CAN2_0xxf6_Tx_Data[8];
uint8_t CAN2_0xxf7_Tx_Data[8];
uint8_t CAN2_0xxf8_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 筛选器编号0-27 | FIFOx | ID类型 | 帧类型
 * @param ID 验证码
 * @param Mask_ID 屏蔽码(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
	
    //检测传参是否正确
    assert_param(hcan != NULL);

	   //CAN过滤器初始化结构体
    CAN_FilterTypeDef can_filter_init_structure;
    //滤波器序号, 0-27, 共28个滤波器
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    
	
    if ((Object_Para & 0x02))
    {   
        //29位 拓展帧
			  // 32位滤波
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
        //验证码 高16bit
        can_filter_init_structure.FilterIdHigh = (ID << 3) >> 16;
        //验证码 低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | (Object_Para & 0x03) << 1;
        //屏蔽码 高16bit
        can_filter_init_structure.FilterMaskIdHigh = (Mask_ID << 3) >> 16;
        //屏蔽码 低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (0x03) << 1 ;
    }
    else
    {
        //11位 标准帧
			  // 32位滤波
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_16BIT;
        //标准帧验证码 高16bit不启用
        can_filter_init_structure.FilterIdHigh = 0x0000 ; 
        //验证码 低16bit
			  can_filter_init_structure.FilterIdLow =ID << 5 | (Object_Para & 0x02) << 4;  
        //标准帧屏蔽码 高16bit不启用
        can_filter_init_structure.FilterMaskIdHigh =  0x0000 ;
        //屏蔽码 低16bit
        can_filter_init_structure.FilterMaskIdLow =(Mask_ID << 5) | 0x01 << 4 ; 
    }

    //滤波器绑定FIFO0或FIFO1
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //从机模式选择开始单元 , 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.SlaveStartFilterBank = 14;
    //使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;

    // 过滤器配置
    if(HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure)!=HAL_OK)
    {
        Error_Handler();
    }
		
		/*离开初始模式*/
 	  HAL_CAN_Start(hcan);				
		
		/*开中断*/
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;					
         can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x200 ,0x7F8);  //只接收0x200-0x207
         can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x200, 0x7F8);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
		can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_EXTID | CAN_DATA_TYPE, 0 ,0);  //只接收
//	    can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_EXTID | CAN_DATA_TYPE, 0x200, 0x7F8);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback()
{
    #ifdef CHASSIS
    
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10 - 1)
    {
        mod10 = 0;
        // CAN2超级电容
        //CAN_Send_Data(&hcan2, 0x220, CAN_Supercap_Tx_Data, 8);
    }

    // CAN1总线  四个底盘电机  
    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);

    // CAN2电机  上板 超电  
    CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8);
    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
	
    #elif defined (GIMBAL)

    // CAN1 摩擦轮*2 pitch
    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);

    // CAN2 yaw 下板
    CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8);
    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);

    #endif
	//测试拓展帧
	// CAN_TxHeaderTypeDef tx_header;
    // uint32_t used_mailbox;

    // tx_header.StdId = 0;
    // tx_header.ExtId = 0x300001B;
    // tx_header.IDE = CAN_ID_EXT;
    // tx_header.RTR = 0;
    // tx_header.DLC = 8;

    // uint8_t Data[8]={0};
    // HAL_CAN_AddTxMessage(&hcan2, &tx_header, Data, &used_mailbox);
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //判断程序初始化完成
    if(init_finished == 0)
    {
        return;
    }
    
    //选择回调函数
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // //判断程序初始化完成, 出问题再加
    // if(init_finished == 0)
    // {
    //     return;
    // }
    
    //选择回调函数
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
