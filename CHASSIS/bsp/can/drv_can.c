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

#include "drv_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

struct Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
struct Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

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


uint8_t CAN2_0x1fe_Tx_Data[8];
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

uint8_t AGV_A_Tx_Data[8];
uint8_t AGV_B_Tx_Data[8];
uint8_t AGV_C_Tx_Data[8];
uint8_t AGV_D_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];




/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x7ff, 0xff,0xffff00,0x1f000000)	标准帧|拓展帧设备ID|拓展帧data2|拓展帧cmd_ID
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    //CAN过滤器初始化结构体
    CAN_FilterTypeDef can_filter_init_structure;

    //检测传参是否正确
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02))
    {
        //拓展帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 << 16;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else
    {
        //数据帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }
    //滤波器序号, 0-27, 共28个滤波器, 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //滤波器绑定FIFO0
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{

    CAN_FilterTypeDef canFilter;
    
    canFilter.FilterBank=1;    																//筛选器组1
    canFilter.FilterIdHigh=0;
    canFilter.FilterIdLow=0;
    canFilter.FilterMaskIdHigh=0;
    canFilter.FilterMaskIdLow=0;
    canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
    canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
    canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
    canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
				
    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        canFilter.SlaveStartFilterBank=14;	

        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_EXTID | CAN_DATA_TYPE, 0x0000001a, 0xff);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_0 | CAN_EXTID | CAN_DATA_TYPE, 0x0000001b, 0xff);
        can_filter_mask_config(hcan, CAN_FILTER(2) | CAN_FIFO_1 | CAN_EXTID | CAN_DATA_TYPE, 0x0000001c, 0xff);
        can_filter_mask_config(hcan, CAN_FILTER(3) | CAN_FIFO_1 | CAN_EXTID | CAN_DATA_TYPE, 0x0000001d, 0xff);




    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
		canFilter.SlaveStartFilterBank=15;						//can2筛选组起始编号
	    //can2筛选组起始编号					
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x206, 0x7ff);
        can_filter_mask_config(hcan, CAN_FILTER(16) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x150, 0x7ff);
        can_filter_mask_config(hcan, CAN_FILTER(17) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x152, 0x7ff);
	  can_filter_mask_config(hcan, CAN_FILTER(18) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x67, 0xff);
    }   
		
	/*配置过滤器*/
	HAL_CAN_ConfigFilter(hcan,&canFilter);
		
	/*离开初始模式*/
	HAL_CAN_Start(hcan);				
	
	/*开中断*/
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
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
 * @brief 配置拓展帧
 *
 * @param equipment_id 设备id
 * @param data2	数据区2内容
 * @param cmd_id	控制指令
 * @return 拓展帧id
 */

uint32_t EXT_ID_Set(uint8_t equipment_id,uint16_t data2,uint8_t cmd_id)
{
	uint32_t send_ext_id;
	send_ext_id	=	cmd_id<<24|data2<<8|equipment_id;
	return send_ext_id;
}
/**
 * @brief 发送拓展帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_EXT_Data(CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.ExtId = ID;
    tx_header.StdId = 0;
    tx_header.IDE = 4;
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
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10 - 1)
    {
        mod10 = 0;
        // CAN1超级电容
//        CAN_Send_Data(&hcan1, 0x220, CAN_Supercap_Tx_Data, 8);
    }

    // CAN1电机
//    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
//    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
//    // CAN_Send_Data(&hcan1, 0x2ff, CAN1_0x2ff_Tx_Data, 8);

//    // CAN2电机
//    CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8);
//向云台发送裁判系统数据
    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
    CAN_Send_Data(&hcan2, 0x1fe, CAN2_0x1fe_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x2ff, CAN2_0x2ff_Tx_Data, 8);
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    
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



