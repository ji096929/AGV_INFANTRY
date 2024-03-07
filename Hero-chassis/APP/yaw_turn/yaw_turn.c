/**
  ******************************************************************************
  * @file    Project/APP/yaw_turn.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件包含了yaw轴的pid参数值，赋值，pid运算，输出等一系列跟拨弹轮有关的函数
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/  

#include "yaw_turn.h"

//接收到的yaw通道值
int16_t yaw_ch2;
//接收到的yaw电流值
int16_t yaw_spin_current;

static void GIMBAL_CALBACK_GET(void);
static void GIMBAL_CALBACK_GET(void);
static void GIMBAL_Set_Mode(void);
static void GIMBAL_Set_Contorl(void);
static void GIMBAL_PID(void);
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_);
static int deadline_judge(float a,int flag);

GIMBAL_t gimbal_y;

float YawGyroPid[5] 	= {0.0f,			0.0f,			0.0f,		0.0f,			0.0f};  //imu角度环
float YawEncondePid[5] 	= {1.5f,			0.0f,			0.0f,		500.0f,			0.0f};  //编码器角度环
float YawEncondePidSpeed[5] 	= {500.0f,			60.0f,			4.0f,		15000.0f,			9000.0f};  //编码器角度环
float YawSpeedPid[5] 			= {2100.0f,			300.0f,			0.0f,		13000.0f,			7000.0f};     //速度环
float yaw_Target_Angle=0.0f;
float	yaw_Target_Speed=0.0f;

//范围转换
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual)
{
	float b=(float)(a-minn_actual)/(float)(maxx_actual-minn_actual);
	float c=(float)b*(float)(maxx-minn);
	float ans=c+(float)minn;
	return ans;
}

void yaw_ch2_to_yaw_target(void)//yaw通道值转变为目标值（仅角度即可）
{
  yaw_Target_Speed=limits_change(CH_YAW_SPEED_MAXX,CH_YAW_SPEED_MINN,yaw_ch2,RC_MAXX,RC_MINN);
	yaw_Target_Angle=limits_change(CH_YAW_ANGLE_MAXX,CH_YAW_ANGLE_MINN,yaw_ch2,RC_MAXX,RC_MINN);
}

uint8_t canTX_yaw(int16_t yaw_current)//yaw轴电流输出
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[2]=yaw_current>>8;
	data[3]=yaw_current&0Xff;
	data[0]=0;
	data[1]=0;
	data[4]=0;
	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

void yaw_turn(void)//yaw轴转动（从接到通道值或电流值到输出电流值）
{
	//UartTX_To_BetaBoard_Yaw_Control(Yaw_Control_Current_Mode, gimbal_y.given_current);
	UartTX_To_BetaBoard_Yaw_Control(Yaw_Control_Target_Angle_Mode, gimbal_y.Target_Angle);
	//canTX_yaw(gimbal_y.given_current);
}

//static void GIMBAL_PID(void)
//{
//	gimbal_motor_encode_pid(&gimbal_y);
//}

//static void GIMBAL_Set_Mode(void)
//{
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
//}

static void GIMBAL_CALBACK_GET(void)
{
	gimbal_y.CAN_actual_angle=yaw_can_rx.angle/8191.0f*360.0f;
	gimbal_y.CAN_Total_Angle=yaw_can_rx.turns*360.0f+gimbal_y.CAN_actual_angle;		//计算yaw的Total_Angle
	gimbal_y.CAN_actual_speed=yaw_can_rx.speed;
}

//static void GIMBAL_Set_Contorl(void)
//{
//	//选定期望值来源(是否自瞄)
////	Vision_Task();
//	 gimbal_y.add_angle=yaw_Target_Angle;//此处有宏定义gimbal_y.add_angle    yaw_angle
//		if(deadline_judge(yaw_angle,1)!=0)
//		{
//			gimbal_y.Target_Angle+=yaw_angle;
//		}

//}

static int deadline_judge(float a,int flag)//遥控器死区判断
{
		if(abs(a)<=YAW_DEADLINE) return 0;
	    else return 1;		
}

float Float_Abs(float a)
{
	if(a<0) return -a;
	else return a;
}

void Yaw_PIDinit(void)//yaw轴pid初始化
{
	PID_Init(&gimbal_y.gimbal_raw_pid,YawSpeedPid);
	PID_Init(&gimbal_y.gimbal_gyro_pid,YawGyroPid);
	PID_Init(&gimbal_y.gimbal_enconde_pid,YawEncondePid);
	PID_Init(&gimbal_y.gimbal_enconde_pid_speed,YawEncondePidSpeed);
}

//云台初始化
//void Gimbal_Init(void)
//{
//	float yaw_Target_Angle=0.0f;
//  float	yaw_Target_Speed=0.0f;
//	int16_t yaw_spin_current=0;
//	gimbal_y.IMU_actual_angle=0.0f;
//	gimbal_y.IMU_actual_speed=0.0f;
//	gimbal_y.CAN_actual_angle=0.0f;
//	gimbal_y.CAN_actual_speed=0.0f;
//	gimbal_y.Target_Angle=72.0f;
//	gimbal_y.Target_Speed=0.0f;
//	gimbal_y.add_angle=0.0f;
//	gimbal_y.given_current=0;
//	
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
//}

static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_)//仅yaw轴
{	
	apid_vpid_realize(&gimbal_->gimbal_enconde_pid,gimbal_->Target_Angle,gimbal_->CAN_Total_Angle);     //pitch的第三个参数为gimbal_->CAN_actual_angle

	gimbal_->Target_Speed=gimbal_->gimbal_enconde_pid.out;

  apid_vpid_realize(&gimbal_->gimbal_enconde_pid_speed,gimbal_->Target_Speed,gimbal_->CAN_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
}


