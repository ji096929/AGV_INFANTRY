/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2022, Hebei, China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin, 
  *                     https://--
  *    
  * FileName   : lk_pitch_turn.c   
  * Version    : v1.0		
  * Author     : NEUQRM, DGYin, 
  * Date       : 2022-11-21         
  * Description: 封装了瓴控Pitch电机的基本操作    
  * Function List:  
  	1. ....
			<version>       : 		
			<modify staff>	:
			<data>          :
			<description>   : 
  	2. ...
  ******************************************************************************
 */
 
/*include----------------------------------------------------------------------*/
//#include "*.h"


#include "bsp_can.h"
#include "relay_task.h"

/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/


/*statement--------------------------------------------------------------------*/
void Yaw_Angle_Process(void);
static int16_t* read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan);
void Encoder_Data_Process(uint8_t *rxdata, Briter_Encoder_t *Encoder, MOTOR_t *Motor);
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);
static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current);
void canTX_To_Gimbal_Yaw_data(int16_t Angle,int16_t Speed);

/*Function prototype Begin*****************************************************/


/*Function prototype End*******************************************************/
/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      ESSESTIAL INFORMATION       ********************
 * ********************                                  ********************
 * **************************************************************************
 *                                                                          *
 *                                   _oo8oo_                                *
 *                                  o8888888o                               *
 *                                  88" . "88                               *
 *                                  (| -_- |)                               *
 *                                  0\  =  /0                               *
 *                                ___/'==='\___                             *
 *                              .' \\|     |// '.                           *
 *                             / \\|||  :  |||// \                          *
 *                            / _||||| -:- |||||_ \                         *
 *                           |   | \\\  -  /// |   |                        *
 *                           | \_|  ''\---/''  |_/ |                        *
 *                           \  .-\__  '-'  __/-.  /                        *
 *                         ___'. .'  /--.--\  '. .'___                      *
 *                      ."" '<  '.___\_<|>_/___.'  >' "".                   *
 *                     | | :  `- \`.:`\ _ /`:.`/ -`  : | |                  *
 *                     \  \ `-.   \_ __\ /__ _/   .-` /  /                  *
 *                 =====`-.____`.___ \_____/ ___.`____.-`=====              *
 *                                   `=---=`                                *
 * **************************************************************************
 * ********************                                  ********************
 * ********************      				 										 ********************
 * ********************         佛祖保佑 永远无BUG       ********************
 * ********************                                  ********************
 * **************************************************************************
 *         .............................................
 *                  佛祖镇楼                  BUG辟易
 *          佛曰:
 *                  写字楼里写字间，写字间里程序员；
 *                  程序人员写程序，又拿程序换酒钱。
 *                  酒醒只在网上坐，酒醉还来网下眠；
 *                  酒醉酒醒日复日，网上网下年复年。
 *                  但愿老死电脑间，不愿鞠躬老板前；
 *                  奔驰宝马贵者趣，公交自行程序员。
 *                  别人笑我忒疯癫，我笑自己命太贱；
 *                  不见满街漂亮妹，哪个归得程序员？
 */

/**
 **************************************************************
 *                                                            *
 *   .=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-.       *
 *    |                     ______                     |      *
 *    |                  .-"      "-.                  |      *
 *    |                 /            \                 |      *
 *    |     _          |              |          _     |      *
 *    |    ( \         |,  .-.  .-.  ,|         / )    |      *
 *    |     > "=._     | )(__/  \__)( |     _.=" <     |      *
 *    |    (_/"=._"=._ |/     /\     \| _.="_.="\_)    |      *
 *    |           "=._"(_     ^^     _)"_.="           |      *
 *    |               "=\__|IIIIII|__/="               |      *
 *    |              _.="| \IIIIII/ |"=._              |      *
 *    |    _     _.="_.="\          /"=._"=._     _    |      *
 *    |   ( \_.="_.="     `--------`     "=._"=._/ )   |      *
 *    |    > _.="                            "=._ <    |      *
 *    |   (_/                                    \_)   |      *
 *    |                                                |      *
 *    '-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-='      *
 *                                                            *
 *           LASCIATE OGNI SPERANZA, VOI CH'ENTRATE           *
 **************************************************************
 */
/*<!--
                       ::
                      :;J7, :,                        ::;7:
                      ,ivYi, ,                       ;LLLFS:
                      :iv7Yi                       :7ri;j5PL
                     ,:ivYLvr                    ,ivrrirrY2X,
                     :;r@Wwz.7r:                :ivu@kexianli.
                    :iL7::,:::iiirii:ii;::::,,irvF7rvvLujL7ur
                   ri::,:,::i:iiiiiii:i:irrv177JX7rYXqZEkvv17
                ;i:, , ::::iirrririi:i:::iiir2XXvii;L8OGJr71i
              :,, ,,:   ,::ir@mingyi.irii:i:::j1jri7ZBOS7ivv,
                 ,::,    ::rv77iiiriii:iii:i::,rvLq@huhao.Li
             ,,      ,, ,:ir7ir::,:::i;ir:::i:i::rSGGYri712:
           :::  ,v7r:: ::rrv77:, ,, ,:i7rrii:::::, ir7ri7Lri
          ,     2OBBOi,iiir;r::        ,irriiii::,, ,iv7Luur:
        ,,     i78MBBi,:,:::,:,  :7FSL: ,iriii:::i::,,:rLqXv::
        :      iuMMP: :,:::,:ii;2GY7OBB0viiii:i:iii:i:::iJqL;::
       ,     ::::i   ,,,,, ::LuBBu BBBBBErii:i:i:i:i:i:i:r77ii
      ,       :       , ,,:::rruBZ1MBBqi, :,,,:::,::::::iiriri:
     ,               ,,,,::::i:  @arqiao.       ,:,, ,:::ii;i7:
    :,       rjujLYLi   ,,:::::,:::::::::,,   ,:i,:,,,,,::i:iii
    ::      BBBBBBBBB0,    ,,::: , ,:::::: ,      ,,,, ,,:::::::
    i,  ,  ,8BMMBBBBBBi     ,,:,,     ,,, , ,   , , , :,::ii::i::
    :      iZMOMOMBBM2::::::::::,,,,     ,,,,,,:,,,::::i:irr:i:::,
    i   ,,:;u0MBMOG1L:::i::::::  ,,,::,   ,,, ::::::i:i:iirii:i:i:
    :    ,iuUuuXUkFu7i:iii:i:::, :,:,: ::::::::i:i:::::iirr7iiri::
    :     :rk@Yizero.i:::::, ,:ii:::::::i:::::i::,::::iirrriiiri::,
     :      5BMBBBBBBSr:,::rv2kuii:::iii::,:i:,, , ,,:,:i@petermu.,
          , :r50EZ8MBBBBGOBBBZP7::::i::,:::::,: :,:,::i;rrririiii::
              :jujYY7LS0ujJL7r::,::i::,::::::::::::::iirirrrrrrr:ii:
           ,:  :@kevensun.:,:,,,::::i:i:::::,,::::::iir;ii;7v77;ii;i,
           ,,,     ,,:,::::::i:iiiii:i::::,, ::::iiiir@xingjief.r;7:i,
        , , ,,,:,,::::::::iiiiiiiiii:,:,:::::::::iiir;ri7vL77rrirri::
         :,, , ::::::::i:::i:::i:i::,,,,,:,::i:i:::iir;@Secbone.ii:::

--*/

void can_filter_set(CAN_FilterTypeDef *CAN_Filter,uint16_t equipment_id,uint8_t cmd_id,uint16_t data2_part,CAN_FILTER_IMPROVE_E 	improve)
{
	if(improve	&	EXT_ID_MODE)
	{
		CAN_Filter->FilterIdHigh = data2_part>>5|cmd_id<<5;
		CAN_Filter->FilterIdLow = equipment_id<<3|data2_part<<11;
		if(improve & EQUIPMENT_ID_IMPROVE)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000;
		CAN_Filter->FilterMaskIdLow =  0xFF<<3;
		}
		else if(improve & EQUIPMENT_ID_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000;
		CAN_Filter->FilterMaskIdLow =  0x0000;
		}
		if(improve &	DATA2_IMPROVE)
		{
		CAN_Filter->FilterMaskIdHigh = 0x07FF|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =  0x1F<<11|CAN_Filter->FilterMaskIdLow;
		}
		else	if	(improve & DATA2_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =  0x0000<<11|CAN_Filter->FilterMaskIdLow;
		}
		
			if(improve &	CMD_ID_IMPROVE)
		{
		CAN_Filter->FilterMaskIdHigh = 0x1F<<11|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =CAN_Filter->FilterMaskIdLow;
		}
		else	if	(improve & CMD_ID_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =CAN_Filter->FilterMaskIdLow;
		}
	}
	else if(improve	& STD_ID_MODE)
	{
		CAN_Filter->FilterIdHigh = equipment_id<<5;
		CAN_Filter->FilterIdLow = 0x0000;
		if(improve & EQUIPMENT_ID_IMPROVE)
		{
		
		CAN_Filter->FilterMaskIdHigh = 0x7FF<<5;
		CAN_Filter->FilterMaskIdLow = 0x0000;
		}
		else if(improve & EQUIPMENT_ID_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000<<5;
		CAN_Filter->FilterMaskIdLow = 0x0000;
		}
	}
	CAN_Filter->FilterBank++;
}

uint8_t bsp_can_init(void)
{
	uint8_t status=0;
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
	canFilter.SlaveStartFilterBank=14;												//can2筛选组起始编号
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//配置过滤器
	canFilter.FilterBank=15;
	can_filter_set(&canFilter,0x1A,03,0,EXT_ID_MODE);
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);
	can_filter_set(&canFilter,0x1B,03,0,EXT_ID_MODE);
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO1;
	can_filter_set(&canFilter,0x1C,03,0,EXT_ID_MODE);
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);
	can_filter_set(&canFilter,0x1D,03,0,EXT_ID_MODE);
	    															//筛选器组15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//配置过滤器
	
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 接收fifo 0不为空中断
	return status;
}

int flag9=0;
uint8_t data[8];
//uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)
//{
//	int i;
//	uint8_t status;
//	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
//	for(i=0;i<8;i++)
//	data[i]=mdata[i];
//	uint32_t  pTxMailbox;
//	if(hcan==&hcan1)
//	{
//		flag9=1;
//		CAN_TxHeaderStruct.StdId=0x200;
//		CAN_TxHeaderStruct.ExtId=0;
//		CAN_TxHeaderStruct.DLC=8;
//		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
//		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
//		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
//	}
//	else if(hcan==&hcan2)
//	{
//		CAN_TxHeaderStruct.StdId=0x007;
//		CAN_TxHeaderStruct.ExtId=0;
//		CAN_TxHeaderStruct.DLC=8;
//		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
//		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
//		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
//	}
//	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
//	return status;
//}
int flag4=0;
float pich_angle;
int mode_now=1;
char M_DAta[5];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN接收函数
{
	flag4=1;
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)//CAN1电调对应回调
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
		gdata=read_motor_data(rxdata,&hcan1);
		angle=gdata[0];
		speed=gdata[1];
		current=gdata[2];
		switch(CAN_RxHeaderStruct.StdId)
		{
			
			case YAW_ID:
				yaw_can_rx.lastangle = yaw_can_rx.angle;
				yaw_can_rx.angle = (int16_t)((rxdata[0]<<8)|rxdata[1]);
				yaw_can_rx.speed = (int16_t)((rxdata[2]<<8)|rxdata[3]);
				Yaw_Angle_Process();
//				canTX_To_Gimbal_Yaw_data(yaw_can_rx.angle,yaw_can_rx.speed);

				break;
			case SHOOT_MOTOR_TRIGGER_ID:
            Trigger_Motor_Callback(&trigger, angle, speed);
        break;
			case MODE_RECEIVE_ID:
				chassis_control_order.chassis_mode		=	(uint8_t)rxdata[0];
				chassis_control_order.Precision_Mode 	= 	(uint8_t)rxdata[1];
				Chassis_Follow_Switch					= 	(uint8_t)rxdata[2];		
				chassis_control_order.last_chassis_mode=chassis_control_order.chassis_mode;
				break;		
			case GIMBAL_CONTROL_ID: //遥控值接收
				chassis_control_order.vx_set=(int16_t)((rxdata[0]<<8)|rxdata[1]); 
				chassis_control_order.vy_set=(int16_t)((rxdata[2]<<8)|rxdata[3]); 
				chassis_control_order.wz_set=(int16_t)((rxdata[4]<<8)|rxdata[5]);	
				break;	
			case Chassis_Shoot_Task_Rx_ID: //是否发射
				if(rxdata[0]==1) 
				{
					shoot_flag=1;
					shoot_angle_clc();
					rxdata[0]=0;
				}
				break;
			
			
		}
	}
	else if(hcan==&hcan2)//此处是CAN2接收云台指令，ID与云台板子已对应
	{
        //防止疯车
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
		{
			
		steering_communication_rx_handler(CAN_RxHeaderStruct.ExtId,rxdata);
		}
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
		{
			
		steering_communication_rx_handler(CAN_RxHeaderStruct.ExtId,rxdata);
		}
	
	}
}
int16_t adata[8];
static int16_t* read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan)//分位读取数据函数
{
//	static int16_t adata[4];
	if(hcan==&hcan1)
	{
		adata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
		adata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
		adata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
		adata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	}
	else if(hcan==&hcan2)//CAN2需全利用
	{
		adata[0]=(int16_t)(rxdata[0]);
		adata[1]=(int16_t)(rxdata[1]);
		adata[2]=(int16_t)(rxdata[2]);
		adata[3]=(int16_t)(rxdata[3]);
		adata[4]=(int16_t)(rxdata[4]);
		adata[5]=(int16_t)(rxdata[5]);
		adata[6]=(int16_t)(rxdata[6]);
		adata[7]=(int16_t)(rxdata[7]);
	}
	return adata;
}

void Encoder_Data_Process(uint8_t *rxdata, Briter_Encoder_t *Encoder, MOTOR_t *Motor)
{
    Encoder->Last_Angle =  Encoder->Absolute_Angle;
	//数据保存
    for (int i=0; i<8; i++)
        Encoder->Received_Data[i] = rxdata[i];
	//角度接收
    Encoder->Raw_Present_Angle = (int) ((Encoder->Received_Data[6]<<24)|(Encoder->Received_Data[5]<<16)|(Encoder->Received_Data[4]<<8)|Encoder->Received_Data[3]);        
	//圈数计算
	Encoder->Raw_Round =  Encoder->Raw_Present_Angle/5120;
    Encoder->Absolute_Angle = (int) ((Encoder->Raw_Present_Angle % 5120) / 5120.0f *81920.0f);
	if (Encoder->Absolute_Angle - Encoder->Last_Angle > 40960)
		Encoder->Total_Round --;
	else if (Encoder->Absolute_Angle - Encoder->Last_Angle < -40960)  
		Encoder->Total_Round ++;
	Encoder->Total_Angle = Encoder->Total_Round*81920 + Encoder->Absolute_Angle; 

		//由于编码器和3508的减速比不同，需要对数据进行转换（运动解算是以3508的减速比得到的角度计算的）
//		Motor->round_cnt =  Encoder->Raw_Present_Angle/512;
//		Motor->last_angle = Motor->actual_angle;
//		Motor->actual_angle = Encoder->Raw_Present_Angle%512 / 512.0f *8191.0f;
//		if(Motor->actual_angle - Motor->last_angle > 4096)h 
//			Motor->round_cnt--;
//		else if (Motor->actual_angle - Motor->last_angle < -4096)
//			Motor->round_cnt++;		
		
		//数据注入
//		Motor->Total_Angle = Motor->round_cnt * 8192 + Motor->actual_angle;
//		Motor->Total_Angle = -Motor->Total_Angle;
		Motor->Total_Angle = -Encoder->Total_Angle;
		//初始位置记录
//		if (Encoder->Request_Flag == 0)
//		{
//			Motor->start_angle = Motor->Total_Angle;
//			while (Motor->start_angle > 81920) Motor->start_angle = Motor->start_angle - 81920;
//			while (Motor->start_angle < 0) Motor->start_angle = Motor->start_angle + 81920;
//		}
		Motor->pid.position_loop.apid.actual_angle =  Motor->Total_Angle;//因为编码器的旋转方向和电机的旋转方向是相反的，所以要进行处理
		Encoder->Request_Flag = 1;
	
    
}
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	//if (All_Initial_Position_Recorded())
	//{
		motor->last_angle = motor->actual_angle;
		motor->actual_angle = angle;
		
		motor->actual_speed = 0.5*(speed + motor->last_speed);
		motor->pid.position_loop.vpid.actual_speed=motor->actual_speed;
		motor->pid.speed_loop.vpid.actual_speed=motor->actual_speed;
		//motor->actual_speed = KalmanFilter(speed,Q,R);
		motor->last_speed = speed;
		motor->actual_current = current;
		//motor1.temp = temp;
//		if(motor->actual_angle - motor->last_angle > 4096)
//			motor->round_cnt--;
//		else if (motor->actual_angle - motor->last_angle < -4096)
//			motor->round_cnt++;
		//motor->Total_Angle = motor->round_cnt * 8192 + motor->actual_angle + motor->start_angle;
		
		//motor->pid.position_loop.apid.actual_angle = motor->Total_Angle;
	//}
}

static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = (float)angle/16384.0f;
	motor->actual_angle*=360.0f;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
//		motor->start_angle = (float)angle/8191.0f*360.0f;
//		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
	
}

void record_yaw_callback(float angle,float speed)
{	
	yaw_can_rx.lastangle = yaw_can_rx.angle;
	yaw_can_rx.angle = angle;
	yaw_can_rx.speed = speed;
	yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)<-180.0f?yaw_can_rx.turns+1:yaw_can_rx.turns;
	yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)>180.0f? yaw_can_rx.turns-1:yaw_can_rx.turns;	
	chassis_center.actual_angle=yaw_can_rx.angle; //用于底盘姿态调整
}


void send_gimbal_data_2(void)
{
	int Angle,Speed;
	Angle=(int16_t)(yaw_can_rx.angle*100); //为了传输小数，将原始数据扩大一百倍
	Speed=(int16_t)yaw_can_rx.speed;
	//canTX_To_Gimbal_Yaw_Callback(Angle, Speed, shoot_flag);
}

//向另一块板子发送裁判系统功率数据
void canTX_RefereeData(int16_t *power,int16_t *buffer,int16_t *max_power)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_RefereeData_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=*power>>8;
	data[1]=*power&0xff;
	data[2]=*buffer>>8;
	data[3]=*buffer&0xff;
	data[4]=*max_power>>8;
	data[5]=*max_power&0xff;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_To_Gimbal_Yaw_Callback(int16_t Angle,int16_t Speed,int16_t shoot_flag_t)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_GIMBAL_Y_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=Angle>>8;
	data[1]=Angle&0xff;
	data[2]=Speed>>8;
	data[3]=Speed&0xff;
	data[4]=shoot_flag_t>>8;
	data[5]=shoot_flag_t&0xff;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_To_Beta_Power_Limit(int Power_Mode)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_Beta_Power_Limit_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=Power_Mode;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_Briter_Encoder(Briter_Encoder_t Paramater,CAN_HandleTypeDef *hcan)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=Paramater.ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=Paramater.Length; //指令长度
	data[1]=Paramater.ID; //现在编码器的ID
	data[2]=Paramater.Order_Code; //指令序号
	for (int i=3; i<8; i++)
        data[i]=Paramater.Send_Data[i];
	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,data,&pTxMailbox);
    status = status;
}

void canTX_To_BetaBoard_WheelVel(void)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
    CAN_TxHeaderStruct.StdId = Chassis_Motor_Speed_ID;
    CAN_TxHeaderStruct.ExtId=0;
    CAN_TxHeaderStruct.DLC=8;
    CAN_TxHeaderStruct.IDE=CAN_ID_STD;
    CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
    CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=(Chassis_Motor1.Target_Speed)>>8;
	data[1]=(Chassis_Motor1.Target_Speed)&0xFF;
	data[2]=(Chassis_Motor2.Target_Speed)>>8;
	data[3]=(Chassis_Motor2.Target_Speed)&0xFF;
	data[4]=(Chassis_Motor3.Target_Speed)>>8;
	data[5]=(Chassis_Motor3.Target_Speed)&0xFF;
	data[6]=(Chassis_Motor4.Target_Speed)>>8;
	data[7]=(Chassis_Motor4.Target_Speed)&0xFF;
	
	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_AGV_Chassis_Motor_Current(void)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
    CAN_TxHeaderStruct.StdId=0x1FF;
    CAN_TxHeaderStruct.ExtId=0;
    CAN_TxHeaderStruct.DLC=8;
    CAN_TxHeaderStruct.IDE=CAN_ID_STD;
    CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
    CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	if (Chassis_MotorA.pid.loop_flag == SPEED_LOOP)
	{
		data[0]=(Chassis_MotorA.pid.speed_loop.vpid.PID_OUT)>>8;
		data[1]=(Chassis_MotorA.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorA.pid.loop_flag == POSITION_LOOP)
	{
		data[0]=(Chassis_MotorA.pid.position_loop.vpid.PID_OUT)>>8;
		data[1]=(Chassis_MotorA.pid.position_loop.vpid.PID_OUT)&0xFF;
	}
	
    if (Chassis_MotorB.pid.loop_flag == SPEED_LOOP)
	{
		data[2]=(Chassis_MotorB.pid.speed_loop.vpid.PID_OUT)>>8;
		data[3]=(Chassis_MotorB.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorB.pid.loop_flag == POSITION_LOOP)
	{
		data[2]=(Chassis_MotorB.pid.position_loop.vpid.PID_OUT)>>8;
		data[3]=(Chassis_MotorB.pid.position_loop.vpid.PID_OUT)&0xFF;
	}
	
    if (Chassis_MotorC.pid.loop_flag == SPEED_LOOP)
	{
		data[4]=(Chassis_MotorC.pid.speed_loop.vpid.PID_OUT)>>8;
		data[5]=(Chassis_MotorC.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorC.pid.loop_flag == POSITION_LOOP)
	{
		data[4]=(Chassis_MotorC.pid.position_loop.vpid.PID_OUT)>>8;
		data[5]=(Chassis_MotorC.pid.position_loop.vpid.PID_OUT)&0xFF;
	}

	if (Chassis_MotorD.pid.loop_flag == SPEED_LOOP)
	{
		data[6]=(Chassis_MotorD.pid.speed_loop.vpid.PID_OUT)>>8;
		data[7]=(Chassis_MotorD.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorD.pid.loop_flag == POSITION_LOOP)
	{
		data[6]=(Chassis_MotorD.pid.position_loop.vpid.PID_OUT)>>8;
		data[7]=(Chassis_MotorD.pid.position_loop.vpid.PID_OUT)&0xFF;
	}
    
    //if (All_Initial_Position_Recorded())
		status=HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeaderStruct,data,&pTxMailbox);
}  
    
void Yaw_Angle_Process(void)
{
	if (yaw_can_rx.angle - yaw_can_rx.lastangle < -4096) yaw_can_rx.turns++;
	if (yaw_can_rx.angle - yaw_can_rx.lastangle >  4096) yaw_can_rx.turns--;
	gimbal_y.CAN_Total_Angle = (float)yaw_can_rx.turns * 8192 + yaw_can_rx.angle;//得到Total_Angle
	float temp;
	temp = gimbal_y.CAN_Total_Angle;
	while (temp > 16384.f) temp =temp - 16384.f; while (temp <0) temp = temp + 16384.f;
	chassis_center.actual_angle=temp*360.0f/16384.0f;
}

void canTX_To_Gimbal_Yaw_data(int16_t Angle,int16_t Speed)
{
	uint8_t data[8]={0};
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_YAW_TX_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=Angle>>8;
	data[1]=Angle&0xff;
	data[2]=Speed>>8;
	data[3]=Speed&0xff;
	
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}