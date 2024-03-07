/**
  ************************************* Copyright ******************************
  *
  *                 (C) Copyright 2022,Hebei,China,NEUQRM.
  *                            All Rights Reserved
  *
  *                     By DGYin,
  *                     https://--
  *
  * FileName   : kinematics.c
  * Version    : v1.1.0-alpha.1
  * Author     : NEUQRM, DGYin,
  * Date       : 2022-11-07
  * Description: 该文件包含了运动学公式，由底盘中心和四个轮子的速度之间的互相转换
  * Function List:
  	1. ....
			<version>				:
			<modify staff>	:
			<data>					:
			<description>		:
  	2. ...
  ******************************************************************************
 */

/*include----------------------------------------------------------------------*/
#include "kinematics.h"

/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/
Kinematics_t Kinematics;

/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/

void Mecanum_Wheel_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Mecanum_Wheel_Get_Base_Velocities(void);
void AGV_Vector_Composition_In_ChassisCoordinate(float linear_x, float linear_y, float angular_z, float Theta, float Motion_Angle);
void DirectiveMotor(float linear_x, float linear_y, float angular_z);
int find_max(void);
void Speed_Limitation(void);

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


/***************************************************************
 * @brief      通过底盘几何中心的三轴速度计算麦轮的速度
 * @author     NEUQRM,
 * @version    1.0.0
 * @date       2022.1.1
 **************************************************************/
void Mecanum_Wheel_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.Wheel_1.Target_Speed.linear_vel = -linear_x - linear_y + angular_z * (half_width + half_length);
	Kinematics.Wheel_2.Target_Speed.linear_vel = -linear_x + linear_y - angular_z * (half_width + half_length);
	Kinematics.Wheel_3.Target_Speed.linear_vel = -linear_x + linear_y + angular_z * (half_width + half_length);
	Kinematics.Wheel_4.Target_Speed.linear_vel = -linear_x -  linear_y - angular_z * (half_width + half_length);    //线速度 cm/s  转转度  RPM
	Kinematics.Wheel_1.Target_Speed.rpm = Kinematics.Wheel_1.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_2.Target_Speed.rpm = Kinematics.Wheel_2.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_3.Target_Speed.rpm = Kinematics.Wheel_3.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_4.Target_Speed.rpm = Kinematics.Wheel_4.Target_Speed.linear_vel * VEL2RPM;
	Chassis_Motor1.Target_Speed = - (int)(Kinematics.Wheel_1.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor2.Target_Speed =   (int)(Kinematics.Wheel_2.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor3.Target_Speed = - (int)(Kinematics.Wheel_3.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor4.Target_Speed =  	(int)(Kinematics.Wheel_4.Target_Speed.rpm * M3508_REDUCTION_RATIO);

}

/***************************************************************
 * @brief      通过底盘几何中心的三轴速度计算各个舵轮牵引电机的速度
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/


void AGV_Vector_Composition_In_ChassisCoordinate(float linear_x, float linear_y, float angular_z, float Theta, float Motion_Angle)
{
	//变量声明
	float Radian_Theta, Radian_Conversion_Angle;
	float X_Components, Y_Components, Sine, Cosine;
	float Gimbal_V, Chassis_Vr; 
	extern int Chassis_Angle;
	
	Gimbal_V = sqrt(linear_x * linear_x + linear_y * linear_y);//云台坐标系的平移速度
	Chassis_Vr = angular_z * (half_width + half_length);//提供底盘角速度的电机速度
	Radian_Theta = Theta/360.00f *(2.00f*3.14f);  //参数中Theta为底盘云台相对角的角度值，要转换为弧度制。
	Motion_Angle = Motion_Angle /8191.0f *(2.00f*3.14f); //参数中Motion_Angle为编码器角度制，要转换为弧度制。但此变量并没有用到。
	
	/*矢量合成*/
	//云台速度和底盘相对角
	Radian_Conversion_Angle = Radian_Theta+atan2(linear_y,linear_x);
	while (Radian_Conversion_Angle >  3.14) Radian_Conversion_Angle  = Radian_Conversion_Angle - 2*3.14;
	while (Radian_Conversion_Angle < -3.14) Radian_Conversion_Angle = Radian_Conversion_Angle + 2*3.14;
	//坐标轴变换
	Sine   = sin(Radian_Conversion_Angle); 
	Cosine = cos(Radian_Conversion_Angle); 
	//修复sin函数不包含坐标轴的问题
	if (linear_y>0 && linear_x==0) Sine   = 1; if (linear_y<0 && linear_x==0) Sine   = -1;
	//if (linear_x>0 && linear_y==0) Cosine = 1; if (linear_x<0 && linear_y==0) Cosine = -1;
	
	/******A转向舵******/
	//矢量合成后X、Y方向分量分别计算。
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  + Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) + Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  + Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) + Chassis_Vr;
	//矢量合成后新矢量的角度。
	Chassis_MotorA.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191;
	//勾股定理计算新矢量长度。
	Kinematics.Wheel_1.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_1.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_1.Target_Speed.linear_vel);
	/*******************/
	/******B转向舵******/
	//矢量合成后X、Y方向分量
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  + Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) + Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  - Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) - Chassis_Vr;
	//向量合成后新向量的角度。
	Chassis_MotorB.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191;
	//勾股定理计算新向量长度。
	Kinematics.Wheel_2.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_2.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_2.Target_Speed.linear_vel);
	/*******************/
	/******C转向舵******/
	//矢量合成后X、Y方向分量
	//矢量合成后X、Y方向分量
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  - Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) - Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  - Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) - Chassis_Vr;
	//向量合成后新向量的角度。
	Chassis_MotorC.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191;
	//勾股定理计算新向量长度。
	Kinematics.Wheel_3.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_3.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_3.Target_Speed.linear_vel);
	/*******************/
	/******D转向舵******/
	//矢量合成后X、Y方向分量
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  - Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) - Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  + Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) + Chassis_Vr;
	//向量合成后新向量的角度。
	Chassis_MotorD.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191;
	//勾股定理计算新向量长度。
	Kinematics.Wheel_4.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_4.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_4.Target_Speed.linear_vel);
	
	/*无矢量合成模式*/
//	Kinematics.Wheel_1.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Kinematics.Wheel_2.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Kinematics.Wheel_3.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Kinematics.Wheel_4.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Chassis_MotorA.ChassisCoordinate_Angle = Chassis_Angle;
//	Chassis_MotorB.ChassisCoordinate_Angle = Chassis_Angle;
//	Chassis_MotorC.ChassisCoordinate_Angle = Chassis_Angle;
//	Chassis_MotorD.ChassisCoordinate_Angle = Chassis_Angle;
	
	//线速度角速度转换。线速度 cm/s  转转度  RPM
	Kinematics.Wheel_1.Target_Speed.rpm = Kinematics.Wheel_1.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_2.Target_Speed.rpm = Kinematics.Wheel_2.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_3.Target_Speed.rpm = Kinematics.Wheel_3.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_4.Target_Speed.rpm = Kinematics.Wheel_4.Target_Speed.linear_vel * VEL2RPM;

}
/***************************************************************
 * @brief      通过遥控器数据和云台底盘相对角，计算以底盘为坐标系的运动方向。
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
float AGV_tan, AGV_ang;
float AGV_DirectiveMotor_RobotMotion_To_TargetStatus(float linear_x, float linear_y, float Theta)
{
	AGV_ang = -atan2(linear_y,linear_x)/3.1415/2* 8191;// - Theta/360.0f * 8191*10;//编码器角度值的角度
    return AGV_ang;
}

/***************************************************************
 * @brief      通过机器人整体的运动方向算各个舵轮转向电机的目标角度。
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/

void AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate(int break_mode)
{
	//计算舵轮的目标角度值
	Chassis_MotorA.Target_Angle = Chassis_MotorA.ChassisCoordinate_Angle + Chassis_MotorA.Zero_Position ; 
	Chassis_MotorB.Target_Angle = Chassis_MotorB.ChassisCoordinate_Angle + Chassis_MotorB.Zero_Position ;
	Chassis_MotorC.Target_Angle = Chassis_MotorC.ChassisCoordinate_Angle + Chassis_MotorC.Zero_Position ;
	Chassis_MotorD.Target_Angle = Chassis_MotorD.ChassisCoordinate_Angle + Chassis_MotorD.Zero_Position ;
	


	
	//刹车模式
	if (break_mode)
	{
		Chassis_MotorA.Target_Angle = 8191*315/360+ Chassis_MotorA.Zero_Position;
		Chassis_MotorB.Target_Angle = 8191*225/360+ Chassis_MotorB.Zero_Position;
		Chassis_MotorC.Target_Angle = 8191*135/360+ Chassis_MotorC.Zero_Position;
		Chassis_MotorD.Target_Angle = 8191*45/360 + Chassis_MotorD.Zero_Position;
		

		
	}
		if(Chassis_MotorA.Target_Angle>8191)Chassis_MotorA.Target_Angle-=8191;
		if(Chassis_MotorA.Target_Angle<0)Chassis_MotorA.Target_Angle+=8191;
		if(Chassis_MotorB.Target_Angle>8191)Chassis_MotorB.Target_Angle-=8191;
		if(Chassis_MotorB.Target_Angle<0)Chassis_MotorB.Target_Angle+=8191;
		if(Chassis_MotorC.Target_Angle>8191)Chassis_MotorC.Target_Angle-=8191;
		if(Chassis_MotorC.Target_Angle<0)Chassis_MotorC.Target_Angle+=8191;
		if(Chassis_MotorD.Target_Angle>8191)Chassis_MotorD.Target_Angle-=8191;
		if(Chassis_MotorD.Target_Angle<0)Chassis_MotorD.Target_Angle+=8191;
		
	
	//记录上次的目标角度值
	Chassis_MotorA.Last_Target_Angle = Chassis_MotorA.Target_Angle;
	Chassis_MotorB.Last_Target_Angle = Chassis_MotorB.Target_Angle;
	Chassis_MotorC.Last_Target_Angle = Chassis_MotorC.Target_Angle;
	Chassis_MotorD.Last_Target_Angle = Chassis_MotorD.Target_Angle;
	
//	Chassis_Motor1.Target_Speed = (int)(Kinematics.Wheel_1.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
//	Chassis_Motor2.Target_Speed = (int)(Kinematics.Wheel_2.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
//	Chassis_Motor3.Target_Speed = (int)(Kinematics.Wheel_3.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
//	Chassis_Motor4.Target_Speed = (int)(Kinematics.Wheel_4.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
	
	Chassis_Motor1.Target_Speed = (int)(Kinematics.Wheel_1.Target_Speed.rpm )*1.0f;
	Chassis_Motor2.Target_Speed = (int)(Kinematics.Wheel_2.Target_Speed.rpm )*1.0f;
	Chassis_Motor3.Target_Speed = (int)(Kinematics.Wheel_3.Target_Speed.rpm )*1.0f;
	Chassis_Motor4.Target_Speed = (int)(Kinematics.Wheel_4.Target_Speed.rpm )*1.0f;

	}
void AAA_AGV_DirectiveMotor_TargetStatus_To_MotorAngle(MOTOR_t *Motor, float Target_Motion_Angle, int Spining_Mode)
{
	//计算舵轮的目标角度值
	Motor->Target_Angle = Target_Motion_Angle + Spining_Mode * 8192*10*315/360  + Motor->Zero_Position; 
	//优弧劣弧处理
	while (Motor->Target_Angle - Motor->Total_Angle >= 8192*5)  Motor->Target_Angle = Motor->Target_Angle - 8192*10;
	while (Motor->Target_Angle - Motor->Total_Angle <= -8192*5) Motor->Target_Angle = Motor->Target_Angle + 8192*10;
	//舵轮旋转优化
	while (Motor->Target_Angle>8192*10) Motor->Target_Angle = Motor->Target_Angle - 8192*10;
	while (Motor->Target_Angle<0) 		Motor->Target_Angle = Motor->Target_Angle + 8192*10;
	while (Motor->Last_Target_Angle>8192*10) Motor->Last_Target_Angle = Motor->Last_Target_Angle - 8192*10;
	while (Motor->Last_Target_Angle<0) 		 Motor->Last_Target_Angle = Motor->Last_Target_Angle + 8192*10;
	if (Float_Abs(Motor->Target_Angle - Motor->Last_Target_Angle) > 8192*2.5f) 
	{
		Motor->Target_Angle = Motor->Target_Angle + 8192*5;
		Motor->Invert_Flag = -Motor->Invert_Flag;
	}
	while (Motor->Target_Angle - Motor->Total_Angle >= 8192*5)  Motor->Target_Angle = Motor->Target_Angle - 8192*10; 
	while (Motor->Target_Angle - Motor->Total_Angle <= -8192*5) Motor->Target_Angle = Motor->Target_Angle + 8192*10;
	//记录上次的目标角度值
	Motor->Last_Target_Angle = Motor->Target_Angle;
	
}
/***************************************************************
 * @brief      通过各个舵轮转向电机的目标角度计算对应的编码器位置。
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
void AGV_DirectiveMotor_MotorAngle_To_EncoderAngle(float Target_Motion_Angle, float Spining_Mode)
{
	Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle ; 
    Chassis_MotorB.Target_Angle = Chassis_MotorB.Target_Angle * 4500 / 360.00f;
    Chassis_MotorC.Target_Angle = Chassis_MotorC.Target_Angle * 4500 / 360.00f;
    Chassis_MotorD.Target_Angle = Chassis_MotorD.Target_Angle * 4500 / 360.00f;
}

/***************************************************************
 * @brief      通过麦轮的实际转速计算底盘几何中心的三轴速度
 * @author     NEUQRM,
 * @version    1.0.0
 * @date       2022.1.1
 **************************************************************/
void Mecanum_Wheel_Get_Base_Velocities(void)
{
    //根据电机转速测算轮子转速
	Kinematics.Wheel_1.actual_speed.rpm = - Chassis_Motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.Wheel_2.actual_speed.rpm =   Chassis_Motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.Wheel_3.actual_speed.rpm =   Chassis_Motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.Wheel_4.actual_speed.rpm = - Chassis_Motor4.actual_speed / M3508_REDUCTION_RATIO;  //轮子转速转换为轮子线速度
	Kinematics.Wheel_1.actual_speed.linear_vel = Kinematics.Wheel_1.actual_speed.rpm * RPM2VEL;
	Kinematics.Wheel_2.actual_speed.linear_vel = Kinematics.Wheel_2.actual_speed.rpm * RPM2VEL;
	Kinematics.Wheel_3.actual_speed.linear_vel = Kinematics.Wheel_3.actual_speed.rpm * RPM2VEL;
	Kinematics.Wheel_4.actual_speed.linear_vel = Kinematics.Wheel_4.actual_speed.rpm * RPM2VEL;  //轮子线速度转换为底盘中心三轴的速度
	Kinematics.actual_velocities.angular_z = ( Kinematics.Wheel_1.actual_speed.linear_vel - Kinematics.Wheel_2.actual_speed.linear_vel\
					- Kinematics.Wheel_3.actual_speed.linear_vel + Kinematics.Wheel_4.actual_speed.linear_vel) / (4.0f * (half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.Wheel_1.actual_speed.linear_vel + Kinematics.Wheel_2.actual_speed.linear_vel\
					- Kinematics.Wheel_3.actual_speed.linear_vel + Kinematics.Wheel_4.actual_speed.linear_vel) / (4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.Wheel_1.actual_speed.linear_vel + Kinematics.Wheel_2.actual_speed.linear_vel\
					+ Kinematics.Wheel_3.actual_speed.linear_vel + Kinematics.Wheel_4.actual_speed.linear_vel) / (4.0f);
}

int find_max(void)
{
	int temp = 0;
	temp = Float_Abs(Chassis_Motor1.Target_Speed);
	if(Float_Abs(Chassis_Motor2.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor2.Target_Speed);
	if(Float_Abs(Chassis_Motor3.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor3.Target_Speed);
	if(Float_Abs(Chassis_Motor4.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor4.Target_Speed);
	return temp;
}

void Speed_Limitation(void)
{
    int temp = 0;
	temp = Float_Abs(Chassis_Motor1.Target_Speed);
	if(Float_Abs(Chassis_Motor2.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor2.Target_Speed);
	if(Float_Abs(Chassis_Motor3.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor3.Target_Speed);
	if(Float_Abs(Chassis_Motor4.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor4.Target_Speed);
	extern int Power_Mode;
	if (Power_Mode == 0) temp = 300;
	//按比例减小速度，防止直线跑歪
    if(temp>MAX_MOTOR_SPEED)
    {
        Chassis_Motor1.Target_Speed=(int)(Chassis_Motor1.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor2.Target_Speed=(int)(Chassis_Motor2.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor3.Target_Speed=(int)(Chassis_Motor3.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor4.Target_Speed=(int)(Chassis_Motor4.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
    }
    
    
}

void Directive_Motor_Angle_Optimize(MOTOR_t *Motor)
{
	float Now_Angle;
	float Target_Angle;
	Target_Angle = Motor->Target_Angle;
	Now_Angle = Motor->Last_Target_Angle;
	//将角度转换为单圈角度
	while(Target_Angle>81920)Target_Angle = Target_Angle-81920; while(Target_Angle<0)Target_Angle = Target_Angle+81920;
	while(Now_Angle>81920)Now_Angle = Now_Angle-81920; while(Now_Angle<0)Now_Angle = Now_Angle+81920;
	//计算角度差值绝对值
	if (Float_Abs(Target_Angle - Now_Angle) < Float_Abs(Now_Angle - Target_Angle))
		Motor->Find_Min = Float_Abs(Target_Angle - Now_Angle);
	else 
		Motor->Find_Min = Float_Abs(Now_Angle - Target_Angle);
	//角度优化
	if (Motor->Find_Min > 8192*2.5f)
	{
		Motor->Target_Angle = Motor->Target_Angle + 8192*5;
		Motor->Invert_Flag = -1;
	}
	else Motor->Invert_Flag = 1;
	
	//优劣弧处理
	while (Motor->Target_Angle - Motor->Total_Angle >= 8192*5)  Motor->Target_Angle = Motor->Target_Angle - 8192*10; while (Motor->Target_Angle - Motor->Total_Angle <= -8192*5) Motor->Target_Angle = Motor->Target_Angle + 8192*10;

}

float Square(float Input) //适用于浮点数的平方函数
{
    float Ans;
    Ans = Input * Input;
    return Ans;
}
