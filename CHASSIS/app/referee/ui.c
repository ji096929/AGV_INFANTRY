//#include "ui.h"
//#include "referee.h"
//#include "usart.h"
//#include	"gimbal_connection.h"

//unsigned char JudgeSend[SEND_MAX_SIZE];
//int Char_Change_Array[7];					//0表示没变化，非0表示有变化
//char LowHP_Flag,lastLowHP_Flag;									//低血量警报
//char Chassis_State,Chassis_lastState;
//ext_student_interactive_char_header_data_t custom_char_draw;
//ext_student_interactive_header_data_t custom_grapic_draw;
//char change_cnt[7];
//float c_pos_x[12] = {0.57,0.34,0.4,0.54,0.3, 0.41,0.64, 0.54,0.40,0.53,0.3,0.4};
//float c_pos_y[12] = {0.65,0.15,0.05,0.1 ,0.1, 0.15,0.1, 0.05,0.1 ,0.15,0.5,0.7};
//float g_pos_x[CAP_GRAPHIC_NUM] = {0.57,0.34,0.4,0.52,0.34,0.42,0.62,0.5,0.42};
//float g_pos_y[CAP_GRAPHIC_NUM] = {0.65,0.15,0.8,0.1,0.1,0.15,0.1,0.8,0.1};
///*瞄准线偏移量*/
//int AIM_bias_x = 0;
//int AIM_bias_y = 0;
//int placece_x[14]={0  , 50, 30,  30, 30,  10,  7,  7,  7,  10,  7,  7,  7 ,10 };
//int placece_y[15]={-80,-320,-80,-100,-120,-140,-160,-180,-200,-220,-240,-260,-280,10, 10 };
//int Graphic_Change_Check(void)
//{
//	int int_AD_actual_value; 	//取整			
//	char vol_num; 
//	int pitch_100;
//	static char last_vol_num;
//	static int last_pitch_100;
//	static char last_color_array[CAP_GRAPHIC_NUM];
//	static int last_Buff_Flag;
//	/*电容数字百分比*/
//	float cap_vol;
//	/*用于初始化基本图形，如车道线，框线，和辅助瞄准线等*/
//	if(connection.connection_rx.Graphic_Init.flag == 0)		
//	{
//		return Op_Init;	//返回Init,会一直发送Add，添加所有图层
//	}
//	
//	/***********************Pitch角度变化检测部分************************/

//	
////		/***********************停车线档位变化检测部分************************/
////	if(pitch_100<-500) park_line = 0;
////	else if(pitch_100 >-500 && pitch_100 < 0) park_line =1;
////	else if(pitch_100 > 0   && pitch_100 < 500) park_line =2;
////	else if(pitch_100 >500  && pitch_100 <1000) park_line =3;
////	else if(pitch_100 >1000 && pitch_100 <1500) park_line =4;
////	else if(pitch_100 >1500 && pitch_100 <2000) park_line =5;
////	else if(pitch_100 >2000 && pitch_100 <3000) park_line =6;
////	
////	if(park_line != last_park_line) park_change_flag = Op_Change;
////	last_park_line = park_line;
//	
////	/***********************Buff状态变化检测部分************************/
////	if(F405.Gimbal_Flag == Gimbal_Buff_Mode)
////		Buff_flag = 1;
////	else
////		Buff_flag = 0;
////	if(Buff_flag != last_Buff_Flag)
////	{
////			buff_change_flag = Op_Change;
////	}
////	last_Buff_Flag = Buff_flag;
//	
//	/*电容电压检测部分*/

//	
//	/*检索有没有发生变化，如果有变化则返回修改图层*/



//	/*都没有变化*/
//	return Op_None;	//返回空操作
//}

//int Char_Change_Check(void)
//{
//	int i;



//	/*用于图形界面初始化*/
//	if(connection.connection_rx.Graphic_Init.flag == 0)		
//	{

//		return Op_Init;	//返回Init,会使一直发送Add，添加所有图层
//	}

//	
//		
//	/*读取云台发送的各种状态*/
//	
//	LowHP_Flag = JudgeReceive.maxHP * 0.35 > JudgeReceive.remainHP ? 1:0;
//	Chassis_State=connection.connection_rx.mode;
//	
//	/*有变化，标志各个位*/

//	if(Chassis_lastState!=Chassis_State)
//	{
//		Char_Change_Array[0]=Op_Change;
//		change_cnt[0]	=2;
//	}
//	
//	if(connection.connection_rx.fric.flag!=connection.connection_rx.fric.last_flag)
//	{
//		Char_Change_Array[1]=Op_Change;
//		change_cnt[1]	=2;
//	
//	}
//		if(connection.connection_rx.follow.flag!=connection.connection_rx.follow.last_flag)
//	{
//		Char_Change_Array[2]=Op_Change;
//		change_cnt[2]	=2;
//	
//	}
//			if(connection.connection_rx.invert.flag!=connection.connection_rx.invert.last_flag)
//	{
//		Char_Change_Array[3]=Op_Change;
//		change_cnt[3]	=2;
//	
//	}
//				if(connection.connection_rx.vision.flag!=connection.connection_rx.vision.last_flag)
//	{
//		Char_Change_Array[4]=Op_Change;
//		change_cnt[4]	=2;
//	
//	}

//	if(LowHP_Flag != lastLowHP_Flag)
//	{
//		Char_Change_Array[5]=Op_Change;
//		change_cnt[5] = 2;	  
//	}
//	
//	/*保存这次标志和上次比较*/

//	lastLowHP_Flag = LowHP_Flag;
//	Chassis_lastState=Chassis_State;
//	
//	
//	/*检索有没有发生变化，如果有变化则返回修改图层*/
//	for(i = 0;i<7;i++)
//	{
//		if(Char_Change_Array[i] == Op_Change)
//			return Op_Change;
//	}
//	
//	return Op_None;	//否则返回空操作，此时不会发送东西
//}

//void referee_data_load_String(int Op_type)
//{
//	static int tick=0;
//	static char Fric_State[2][6] = {"CLOSE","OPEN"};
//	static char Vision_State[2][6] = {"CLOSE","OPEN"};
//	static char Chassis_State[4][9] = {"NOForce","Normal","SPIN","PRECISION"};
//	static char Gimbal_State[4][9] = {"NOForce","Normal","SPIN","PRECISION"};
//	static char Power_State[2][4] = {"Bat","Cap"};

//	/*初始化操作，轮流生成图层*/
//	if(Op_type == Op_Init)
//	{
//		switch(tick%11)
//		{
//			/*静态字符*/
//			case 0:
//			/*******************************pitch 字符*********************************/
//			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
//			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
//			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
//			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
//			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
//			custom_char_draw.char_custom.grapic_data_struct.layer=9;
//			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
//			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
//			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("PITCH:");
//			custom_char_draw.char_custom.grapic_data_struct.width=2;
//			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[0]*SCREEN_LENGTH;
//			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[0]*SCREEN_WIDTH;
//			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
//			strcpy(custom_char_draw.char_custom.data,"PITCH:");
//			break;

//		
//			default:
//			break;
//		}
//		tick++;
//		return ;
//		
//	}else if(Op_type == Op_Change)		//如果是标志为修改
//	{
//		/*寻找是哪个发生了变化*/
//	
//		
//		if(Char_Change_Array[5] == Op_Change)  
//		{
//			if(change_cnt[5]>0)
//			{
//			 change_cnt[5] -- ;
//			}
//			else
//			{
//			Char_Change_Array[5] = Op_None;
//			}
//			/*******************************绘制删除 RUN 字符*********************************/
//			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
//			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
//			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 10;
//			custom_char_draw.char_custom.grapic_data_struct.operate_tpye= (LowHP_Flag == 1 ?Op_Add : Op_Delete);
//			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
//			custom_char_draw.char_custom.grapic_data_struct.layer=8;
//			custom_char_draw.char_custom.grapic_data_struct.color= Orange;
//			custom_char_draw.char_custom.grapic_data_struct.start_angle=50;    //字体大小  
//			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("RUN !!");
//			custom_char_draw.char_custom.grapic_data_struct.width=6;           //宽度
//			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[10]*SCREEN_LENGTH;
//			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[10]*SCREEN_WIDTH;
//			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
//			strcpy(custom_char_draw.char_custom.data,"RUN !!");
//		}
//		
//	
//	}
//}

//void referee_data_load_Graphic(int Op_type)
//{
//	static int pack_tick = 0;			//数据包计数器
//	static unsigned int pitch = 0;
//	static float Start_Place_x[CAP_GRAPHIC_NUM] = {0.388,0.41,0.432,0.454,0.476,0.498,0.52,0.542,0.564};
//  static float Start_Place_y = 0.02;
//	static float End_Place_x[CAP_GRAPHIC_NUM] = {0.398,0.42,0.442,0.464,0.486,0.508,0.53,0.552,0.574};
//	static float End_Place_y = 0.04;
//	static int i;
//	int packed_tick = 0;							//装包计数器
//	/*初始化操作，轮流生成图层*/
//	if(Op_type == Op_Init)
//	{
//		switch(pack_tick % PACK_NUM)
//		{
//		case 0:



//						/*******************************pitch角度更新*********************************/
//PITCH_:	
//				/*******************************Pitch 浮点数*********************************/
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].graphic_name[1] = 3;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].graphic_name[2] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].graphic_tpye=5;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].layer=3;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].color=Pink;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].start_angle=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].end_angle=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].width=4;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].start_x=0.65 * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].start_y=0.65 * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].radius = pitch & 0x03ff;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].end_x  = (pitch >> 10) & 0x07ff;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:0].end_y  = (pitch >> 21) & 0x07ff;
////				if(Op_type == Op_Change) goto CONT_2;
//				break;
//			default:
//				break;
//		}
//		pack_tick++;
//	}
//	else if(Op_type == Op_Change)
//	{

////		if(pitch_change_flag == Op_Change )
////		{
////			 goto PITCH_;
////			 CONT_2:pitch_change_flag = Op_None;
////		}

//	}
//}

//void JudgementCustomizeChar(int Op_type)
//{
//		custom_char_draw.data_cmd_id=0x0110;//绘制字符

//		custom_char_draw.sender_ID=JudgeReceive.robot_id;//发送者ID，机器人对应ID
//		if(JudgeReceive.robot_id == 103)
//				custom_char_draw.receiver_ID = 0x0167;
//		if(JudgeReceive.robot_id == 104)
//				custom_char_draw.receiver_ID = 0x0168;
//		if(JudgeReceive.robot_id == 105)
//				custom_char_draw.receiver_ID = 0x0169;
//		if(JudgeReceive.robot_id == 3)
//				custom_char_draw.receiver_ID = 0x0103;	
//		if(JudgeReceive.robot_id == 4)
//				custom_char_draw.receiver_ID = 0x0104;
//		if(JudgeReceive.robot_id == 5)
//				custom_char_draw.receiver_ID = 0x0105;

///*********************************自定义字符数据***********************************/
//		referee_data_load_String(Op_type);
//}

//void JudgementCustomizeGraphics(int Op_type)
//{
//		custom_grapic_draw.data_cmd_id=0x0104;//绘制七个图形（内容ID，查询裁判系统手册）

//		custom_grapic_draw.sender_ID=JudgeReceive.robot_id;//发送者ID，机器人对应ID
//		if(JudgeReceive.robot_id == 103)
//				custom_grapic_draw.receiver_ID = 0x0167;
//		if(JudgeReceive.robot_id == 104)
//				custom_grapic_draw.receiver_ID = 0x0168;
//		if(JudgeReceive.robot_id == 105)
//				custom_grapic_draw.receiver_ID = 0x0169;
//		if(JudgeReceive.robot_id == 3)
//				custom_grapic_draw.receiver_ID = 0x0103;	
//		if(JudgeReceive.robot_id == 4)
//				custom_grapic_draw.receiver_ID = 0x0104;
//		if(JudgeReceive.robot_id == 5)
//				custom_grapic_draw.receiver_ID = 0x0105;

///*********************************自定义图像数据***********************************/
//		referee_data_load_Graphic(Op_type);
//}

//uint8_t seq = 0;
//void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
//{
//	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

//	memset(JudgeSend,0,frame_length);  //存储数据的数组清零
//	
//	/*****帧头打包*****/
//	JudgeSend[0] = sof;//数据帧起始字节
//	memcpy(&JudgeSend[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
//	JudgeSend[3] = seq;//包序号
//	Append_CRC8_Check_Sum(JudgeSend,frameheader_len);  //帧头校验CRC8

//	/*****命令码打包*****/
//	memcpy(&JudgeSend[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
//	
//	/*****数据打包*****/
//	memcpy(&JudgeSend[frameheader_len+cmd_len], p_data, len);
//	Append_CRC16_Check_Sum(JudgeSend,frame_length);  //一帧数据校验CRC16

//	if (seq == 0xff) seq=0;
//  else seq++;
//	
//	/*****数据上传*****/
//		while(HAL_DMA_GetState(&hdma_usart6_tx)); 

//		hdma_usart6_tx.Instance->NDTR = frame_length; 

//		HAL_DMA_Start(&hdma_usart6_tx, (uint32_t)JudgeSend, (uint32_t)&huart6.Instance->DR, frame_length); // ??DMA??
//	
//}

//int char_change_state,graphic_change_state;

//void UI_Send_Graphic_Task(void)
//{
//			graphic_change_state = Graphic_Change_Check();
//			if(graphic_change_state)
//			{
//				JudgementCustomizeGraphics(graphic_change_state);
//				if(graphic_change_state != Op_None)
//				referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw));
//			}
//}

//void UI_Send_Char_Task(void)
//{
//		char_change_state = Char_Change_Check();
//			if(char_change_state)			//检查有没有变化，没有变化就不发，节省带宽
//			{
//				JudgementCustomizeChar(char_change_state);
//				if(char_change_state != Op_None)
//					referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_char_draw,sizeof(custom_char_draw));
//			}
//}