/**
  ************************************* Copyright ******************************
  *
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *
  *                     By DGYin,
  *                     https://--
  *
  * FileName   : chassis_move.c
  * Version    : v1.1.0-alpha.1
  * Author     : NEUQRM, DGYin,
  * Date       : 2022-11-07
  * Description: 该文件是针对得到的模式来进行对应的输出和底盘功率限制
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
#include "chassis_move.h"
#include "kinematics.h"
#include "bsp_can.h"
#include "supercap.h"
#include "steering_communication.h"
#include "chassis_power_control.h"
/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/

/***********************************************************
*@Brief	供外部调用的变量
***********************************************************/
int Chassis_Follow_Switch = Chassis_Follow_ON;

float total_current_limit, total_current, power;
uint16_t buffer, max_power;
BUFFER_PID_t b_pid;
float current_scale;
int test_speed;
CHASSIS_CONTROL_ORDER_t chassis_control_order;
MOTOR_t Chassis_Motor1, Chassis_Motor2, Chassis_Motor3, Chassis_Motor4, chassis_center, Chassis_MotorA, Chassis_MotorB, Chassis_MotorC, Chassis_MotorD;
POWER_PID_t p_pid;
int shoot_flag = 0;

POWER_PID_t p_pid;
BUFFER_PID_t b_pid;
REAl_CHASSIS_SPEED_t real_chassis_speed;
int8_t max_d_speed_x;
int8_t max_d_speed_y;
float avx, avy, awz;
float last_vx, last_vy;

steering_communication_pack_t pack;
//int Chassis_Type;
/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/
void chassis_spin(float *vx, float *vy);
static float chassis_follow(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r, float Theta);
static void chassis_move_mode(void);
static void can_send_chassis_current(void);
static float Get_chassis_theta(void);
static void power_limitation_jugement(void);
static void power_limitation_jugement(void);
static float chassis_buffer_loop(uint16_t buffer);
static void chassis_fly(uint16_t buffer);
static float chassis_power_loop(uint16_t target_power, float actual_power, float last_power);
void Power_Loop(float Power, int Max_Power);
void Buffer_Loop(void);
void Cap_Volt_Loop(void);
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


/**原readme
  ******************************************************************************
  * @file    Project/APP/chassis_move.c
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件是针对得到的模式来进行对应的输出和底盘功率限制
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

void chassis_move(void)//底盘移动过程
{

    chassis_move_mode(); //对应模式的底盘状态赋值	
    
}

void AGV_connoection(int ms_cnt)
{
	 if(chassis_power_control.all_mscb_ready_flag&0x01	&& ms_cnt%5==0)
    {
        if (chassis_control_order.chassis_mode != 3)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                steering_communication_SET_VELOCITY_VECTOR(A_MscB, Chassis_MotorA.Target_Angle, Chassis_Motor1.Target_Speed, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY, pack);       
            }
            else
            {
							 steering_communication_SET_VELOCITY_VECTOR(A_MscB, Chassis_MotorA.Target_Angle, Chassis_Motor1.Target_Speed, chassis_power_control.scaled_power_32[0], pack);               
            }
        }
        else
        {
								steering_communication_SET_VELOCITY_VECTOR(A_MscB, Chassis_MotorA.Target_Angle, 0, chassis_power_control.scaled_power_32[0], pack);              
        }

        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x0E;
    }
		 if(chassis_power_control.all_mscb_ready_flag&0x02	&& ms_cnt%5==1)
    {
        if (chassis_control_order.chassis_mode != 3)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                steering_communication_SET_VELOCITY_VECTOR(B_MscB, Chassis_MotorB.Target_Angle, Chassis_Motor2.Target_Speed, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY, pack);       
            }
            else
            {
							 steering_communication_SET_VELOCITY_VECTOR(B_MscB, Chassis_MotorB.Target_Angle, Chassis_Motor2.Target_Speed, chassis_power_control.scaled_power_32[1], pack);               
            }
        }
        else
        {
								steering_communication_SET_VELOCITY_VECTOR(B_MscB, Chassis_MotorB.Target_Angle, 0, chassis_power_control.scaled_power_32[1], pack);              
        }

        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x0D;
    }
		 if(chassis_power_control.all_mscb_ready_flag&0x04	&& ms_cnt%5==2)
    {
        if (chassis_control_order.chassis_mode != 3)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                steering_communication_SET_VELOCITY_VECTOR(C_MscB, Chassis_MotorC.Target_Angle, Chassis_Motor3.Target_Speed, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY, pack);       
            }
            else
            {
							 steering_communication_SET_VELOCITY_VECTOR(C_MscB, Chassis_MotorC.Target_Angle, Chassis_Motor3.Target_Speed, chassis_power_control.scaled_power_32[2], pack);               
            }
        }
        else
        {
								steering_communication_SET_VELOCITY_VECTOR(C_MscB, Chassis_MotorC.Target_Angle, 0, chassis_power_control.scaled_power_32[2], pack);              
        }

        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x0B;
    }
		 if(chassis_power_control.all_mscb_ready_flag&0x08	&& ms_cnt%5==3)
    {
        if (chassis_control_order.chassis_mode != 3)
        {
            chassis_power_control.set_vector_speed_only_cnt++;
            if (chassis_power_control.set_vector_speed_only_cnt <= DEFAULT_SET_VECTOR_SPEED_ONLY_CNT)
            {
                steering_communication_SET_VELOCITY_VECTOR(D_MscB, Chassis_MotorD.Target_Angle, Chassis_Motor4.Target_Speed, VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY, pack);       
            }
            else
            {
							 steering_communication_SET_VELOCITY_VECTOR(D_MscB, Chassis_MotorD.Target_Angle, Chassis_Motor4.Target_Speed, chassis_power_control.scaled_power_32[3], pack);               
            }
        }
        else
        {
								steering_communication_SET_VELOCITY_VECTOR(D_MscB, Chassis_MotorD.Target_Angle, 0, chassis_power_control.scaled_power_32[3], pack);              
        }

        chassis_power_control.all_mscb_ready_flag = chassis_power_control.all_mscb_ready_flag&0x07;
    }
		
	
}

void vpid_chassis_realize_F(void)
{
    switch_flag = CHASSIS;
    //PID运算
    pid_realize(&(Chassis_Motor1.pid));
    pid_realize(&(Chassis_Motor2.pid));
    pid_realize(&(Chassis_Motor3.pid));
    pid_realize(&(Chassis_Motor4.pid));
    if (Chassis_Type == AGV_Chassis)
    {

        Chassis_MotorA.pid.position_loop.apid.Target_Angle = Chassis_MotorA.Target_Angle;
        Chassis_MotorB.pid.position_loop.apid.Target_Angle = Chassis_MotorB.Target_Angle;
        Chassis_MotorC.pid.position_loop.apid.Target_Angle = Chassis_MotorC.Target_Angle;
        Chassis_MotorD.pid.position_loop.apid.Target_Angle = Chassis_MotorD.Target_Angle;
        pid_realize(&(Chassis_MotorA.pid));
        pid_realize(&(Chassis_MotorB.pid));
        pid_realize(&(Chassis_MotorC.pid));
        pid_realize(&(Chassis_MotorD.pid));
    }

    switch_flag = NUL;
}

int Chassis_Angle;
static void chassis_speed_control(float speed_x, float speed_y, float speed_r, float Theta)
{
	//计算底盘为坐标系的底盘目标角度
	Chassis_Angle = AGV_DirectiveMotor_RobotMotion_To_TargetStatus(speed_x, speed_y, Theta);
	//刹车模式
	int break_mode = 0;
	if ((speed_x == 0) && (speed_y == 0) && (speed_r == 0))
		break_mode = 1;
	//计算动力电机的速度
	AGV_Vector_Composition_In_ChassisCoordinate(speed_x, speed_y, speed_r, Theta, Chassis_Angle);
	//矢量合成
	
	//计算底盘为坐标系的转向电机的角度
	AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate(break_mode);

	//速度限制
    Speed_Limitation();
	
	
	
   

}

static float chassis_follow(void)
{
    chassis_center.pid.position_loop.apid.Target_Angle = GIMBAL_HEAD_ANGLE + 180.0f*gimbal_y.Invert_Flag; //零点位置
    chassis_center.pid.position_loop.apid.actual_angle = chassis_center.actual_angle;
    follow_pid_realize();
    return (float)chassis_center.pid.position_loop.apid.PID_OUT;
}

/***********************************************************
*@fuction	:Target_Velocity_Smoothen
*@brief		:加速度曲线控制，有利于功率控制
*@param		:--
*@return	:void
*@author	:DGYin
*@date		:2023-04-14
***********************************************************/

void Target_Velocity_Smoothen(int Target_Speed, int Current_Speed, int Smoothen_Method)
{
    if (Smoothen_Method == Uniform_Acceleration)
    {
        if (Target_Speed > Current_Speed)
            if (Target_Speed - Current_Speed > Acceleration)
                Target_Speed = Target_Speed + Acceleration;
        if (Target_Speed < Current_Speed)
            if (Current_Speed - Target_Speed > Acceleration)
                Target_Speed = Target_Speed - Acceleration;
    }
}

/***********************************************************
*@fuction	:chassis_move_mode
*@brief		:结算对应模式下的底盘动作
*@param		:--
*@return	:void
*@author	:DGYin
*@date		:2023-04-13
***********************************************************/
float vx, vy, wz;
float Gimbal_Chassis_Relative_Angle, Relative_Angle_For_UI;
float Get_Gimbal_Chassis_Relative_Angle(void);
static void chassis_move_mode(void)
{
	if(gimbal_y.Invert_Flag)
		{
		
	vx = (float)chassis_control_order.vx_set;
	vy = (float)chassis_control_order.vy_set;
	wz = (float)chassis_control_order.wz_set/10.0f;
	}
		else
		{
		
		vx = -(float)chassis_control_order.vx_set;
			vy =-(float)chassis_control_order.vy_set;
				wz =-(float)chassis_control_order.wz_set/10.0f;
		}
   
	Gimbal_Chassis_Relative_Angle = Get_Gimbal_Chassis_Relative_Angle();//Gimbal_Chassis_Relative_Angle以云台为0度，CW为0~180°，CCW为0~-180°。
    Relative_Angle_For_UI = Gimbal_Chassis_Relative_Angle;
	//	chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;
	switch(chassis_control_order.chassis_mode)
	{
		//模式为底盘无力时
		case CHASSIS_REMOTE_CLOSE:
			vx = 0;
			vy = 0;
			wz = 0;
			break;
		//云台随动模式时
		case CHASSIS_NORMAL:
			if (Chassis_Follow_Switch == Chassis_Follow_ON)		wz = wz -1.0f * chassis_follow(); //为了使底盘回到云台角度而产生的旋转
			if (Chassis_Follow_Switch == Chassis_Follow_OFF)	Gimbal_Chassis_Relative_Angle = 0;
			break;
		//旋转模式时
		case CHASSIS_SPIN:
			if (vx==0 && vy==0)
				wz = 3.0f;
			else wz = 1.0f;
			break;
	}
	
    chassis_speed_control(vx, vy, wz, Gimbal_Chassis_Relative_Angle);

}

float Get_Gimbal_Chassis_Relative_Angle(void)
{
	float Chassis_Angle;
	float Gimbal_Angle;
	Gimbal_Angle = GIMBAL_HEAD_ANGLE + 180.0f*gimbal_y.Invert_Flag; //通过编码器设置的零点为云台角度
	Chassis_Angle = chassis_center.actual_angle;
	if (Chassis_Angle - Gimbal_Angle > 180.f) return Chassis_Angle - Gimbal_Angle-360;
	if (Chassis_Angle - Gimbal_Angle < -180.f) return Chassis_Angle - Gimbal_Angle+360;
	return Chassis_Angle - Gimbal_Angle;
}
static void can_send_chassis_current(void)//输出底盘电流值
{
	//发送转向电机的目标电流值
	canTX_AGV_Chassis_Motor_Current();
	//发送动力电机的目标速度值
	if (All_Directive_Motor_Angle_Ready())//转向电机角度到位，才能开始移动
	{
		//UartTX_To_BetaBoard_WheelVel(Chassis_Motor1.Target_Speed, Chassis_Motor2.Target_Speed, Chassis_Motor3.Target_Speed, Chassis_Motor4.Target_Speed);
		canTX_To_BetaBoard_WheelVel();
	}
}

float theta;
void chassis_spin(float *vx, float *vy)
{
    theta = Get_chassis_theta();
	avy = *vy; avx = *vx;
    *vx = (float)(avy * sin(theta) + avx * cos(theta));
    *vy = (float)(avy * cos(theta) - avx * sin(theta));
}

static float Get_chassis_theta(void)//读取底盘相对云台转角（需要云台陀螺仪）
{
    float temp, Chassis_Center_Actual_Angle, Theta;
    if(chassis_center.actual_angle < chassis_center.switch_mode_angle)
        Chassis_Center_Actual_Angle = chassis_center.actual_angle + 360.0f;
    else
        Chassis_Center_Actual_Angle = chassis_center.actual_angle;
    temp = Chassis_Center_Actual_Angle - chassis_center.switch_mode_angle;
    Theta = temp / 360.0f * 2 * PI;
    return Theta;
}

/***********************************************************
*@fuction	:Cap_Volt_Loop
*@brief		:
*@param		:--
*@return	:void
*@author	:DGYin
*@date		:2023-05-16
***********************************************************/

void Cap_Volt_Loop(void)
{
	float Scale1=1, Scale2=1;
	extern int supercap_volt;
	//supercap_volt = 240;
	if (supercap_volt<180)
		Scale2 = (supercap_volt-130)/50.f;
	if (Scale2<0) Scale2 = 0;
	Chassis_Motor1.Target_Speed =  Chassis_Motor1.Target_Speed*Scale1*Scale2;
	Chassis_Motor2.Target_Speed =  Chassis_Motor2.Target_Speed*Scale1*Scale2;
	Chassis_Motor3.Target_Speed =  Chassis_Motor3.Target_Speed*Scale1*Scale2;
	Chassis_Motor4.Target_Speed =  Chassis_Motor4.Target_Speed*Scale1*Scale2;
}


/***********************************************************
*@fuction	:Buffer_Loop
*@brief		:
*@param		:--
*@return	:void
*@author	:DGYin
*@date		:2023-05-16
***********************************************************/
fp32 power;
uint16_t  buffer, max_power;
void Buffer_Loop(void)
{

	float Scale1=1, Scale2;
	get_chassis_power_and_buffer_and_max(&power, &buffer, &max_power);
//	if (power>max_power && buffer<60)
//		Scale1 = 0.9;
//	else Scale1 = 1;
	Scale2 = (buffer-10)/60.f;
	if (Scale2<0) Scale2 = 0;
	Chassis_Motor1.Target_Speed =  Chassis_Motor1.Target_Speed*Scale1*Scale2;
	Chassis_Motor2.Target_Speed =  Chassis_Motor2.Target_Speed*Scale1*Scale2;
	Chassis_Motor3.Target_Speed =  Chassis_Motor3.Target_Speed*Scale1*Scale2;
	Chassis_Motor4.Target_Speed =  Chassis_Motor4.Target_Speed*Scale1*Scale2;
	

}
/**
  * @breif         麦轮时代的底盘功率限制
  * @param[in]     none
	* @param[out]    输出限制后的四个电机电流值
  * @retval        none
  */
float current_scale, BUFFER_MAX = 60.0f, POWER_TOTAL_CURRENT_LIMIT = 9000.0f;
float temp3, temp1, temp2, speed1, speed2, speed3, speed4, total_current_limit, total_current, power, last_power;
float power_scale, buffer_scale;
uint8_t fly_flag;
static void power_limitation_jugement(void)
{
    total_current = 0;
    last_power = power;
    get_chassis_power_and_buffer_and_max(&power, &buffer, &max_power);
    canTX_RefereeData(&power, &buffer, &max_power);

    power_scale = chassis_power_loop(max_power - 10, power, last_power);
    buffer_scale = chassis_buffer_loop(buffer);
    temp1 = Chassis_Motor1.Target_Speed * buffer_scale * power_scale;
    Chassis_Motor1.Target_Speed = (int16_t)temp1;
    temp1 = Chassis_Motor2.Target_Speed * buffer_scale * power_scale;
    Chassis_Motor2.Target_Speed = (int16_t)temp1;
    temp1 = Chassis_Motor3.Target_Speed * buffer_scale * power_scale;
    Chassis_Motor3.Target_Speed = (int16_t)temp1;
    temp1 = Chassis_Motor4.Target_Speed * buffer_scale * power_scale;
    Chassis_Motor4.Target_Speed = (int16_t)temp1;
    //	if(power>max_power*2)CHASSIS_vPID_max=5000;
    //	else if(power<max_power*1.6)CHASSIS_vPID_max=8000;
    if(buffer < BUFFER_MAX * 0.15f) CHASSIS_vPID_max = 500;
    if(buffer < BUFFER_MAX * 0.25f && buffer >= BUFFER_MAX * 0.15f) CHASSIS_vPID_max = 900;
    if(buffer < BUFFER_MAX * 0.36f && CHASSIS_vPID_max <= 8000) CHASSIS_vPID_max = 1200; //原来是1200
    if(CHASSIS_vPID_max <= 1900 && buffer >= BUFFER_MAX * 0.55f ) CHASSIS_vPID_max = 1600;
    if(CHASSIS_vPID_max <= 2100 && buffer >= BUFFER_MAX * 0.75f ) CHASSIS_vPID_max = 3200;
    if(CHASSIS_vPID_max <= 4500 && buffer >= BUFFER_MAX * 0.96f ) CHASSIS_vPID_max = 8000;
}

/**
  * @breif         麦轮时代的底盘功率环函数
  * @param[in]     target_power：设定的目标值
	* @param[in]     target_power：返回的真实值
	* @param[in]     last_power：上一次返回的真实值
	* @param[out]    四个电机的输出电流
  * @retval        none
  */
static float chassis_power_loop(uint16_t target_power, float actual_power, float last_power)
{
    float temp;
    p_pid.target_power = (float)target_power;
    p_pid.actual_power = actual_power;
    //此处进行pid运算
    power_pid_realize(&p_pid);
    //此处计算比例系数
    temp = 1.07f + ((float)p_pid.PID_OUT / 1000);
    //	if(temp>1.2f)  temp-=0.2f;
    //	temp=temp/2.0*0.45f+0.1f;
    temp *= 0.7f;

    return temp;
}

static float chassis_buffer_loop(uint16_t buffer)
{
    float temp;
    b_pid.target_buffer = 50;
    b_pid.actual_buffer = buffer;
    buffer_pid_realize(&b_pid);
    temp = 1.07f - ((float)b_pid.PID_OUT / 1000.0f);
    temp *= 0.75f;
    //	if(temp>1.2f)  temp-=0.2f;

    return temp;
}

/**
  * @breif         底盘飞坡函数，防止因飞坡后缓冲能量用完
  * @param[in]     buffer：底盘缓冲能量
	* @param[out]    四个电机的输出电流
  * @retval        none
  */
static void chassis_fly(uint16_t buffer)
{
    if(buffer < 20)
    {
        Chassis_Motor1.pid.speed_loop.vpid.PID_OUT *= 0.5f;
        Chassis_Motor2.pid.speed_loop.vpid.PID_OUT *= 0.5f;
        Chassis_Motor3.pid.speed_loop.vpid.PID_OUT *= 0.5f;
        Chassis_Motor4.pid.speed_loop.vpid.PID_OUT *= 0.5f;
    }
}

/***********************************************************
*@fuction	:Motor_Power_Prosumption_Calculate
*@brief		:通过读取电机转速与力矩预测功率，具体请阅“软件底盘功率限制解读.md”，暂时没啥用
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-03
***********************************************************/

void Motor_Power_Prosumption_Calculate(MOTOR_t *Motor)
{
    //根据电机功率式，预测电机功率
    Motor->Target_Torque = abs(Tau_Coefficient * Motor->target_current * 20.0f / 16384.0f);
    Motor->Actual_Torque = abs(Tau_Coefficient * Motor->actual_current * 20.0f / 16384.0f);
    Motor->Power_Prosumption = Motor->Target_Torque *  Motor->Target_Speed; //输出功率
    Motor->Power_Prosumption = Motor->Power_Prosumption + Power_K1 * Motor->Target_Torque * Motor->Target_Torque + Power_K2 * Motor->Target_Speed * Motor->Target_Speed; //实际输入功率
    Motor->Power_Prosumption = Motor->Power_Prosumption * Power_Prosumption_Converting_Coefficient; //单位转换
}

/***********************************************************
*@fuction	:Motor_Power_Prosumption
*@brief		:调用Motor_Power_Prosumption_Calculate，实现功率预测
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-03
***********************************************************/

void Motor_Power_Prosumption(void)
{
    Motor_Power_Prosumption_Calculate(&Chassis_Motor1);
    Motor_Power_Prosumption_Calculate(&Chassis_Motor2);
    Motor_Power_Prosumption_Calculate(&Chassis_Motor3);
    Motor_Power_Prosumption_Calculate(&Chassis_Motor4);
    Motor_Power_Prosumption_Calculate(&Chassis_MotorA);
    Motor_Power_Prosumption_Calculate(&Chassis_MotorB);
    Motor_Power_Prosumption_Calculate(&Chassis_MotorC);
    Motor_Power_Prosumption_Calculate(&Chassis_MotorD);

}



/***********************************************************
*@fuction	:Motor_Power_Limitation
*@brief		:计算功率总和，进行功率限制
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-03
***********************************************************/

void Motor_Power_Limitation(void)
{
    int Chassis_Gross_Power_Prosumption = 0;
    int Chassis_Traction_Motor_Power_Prosumption = 0;
    int Chassis_Directive_Motor_Power_Prosumption = 0;
    Chassis_Traction_Motor_Power_Prosumption  = Chassis_Motor1.Power_Prosumption + Chassis_Motor2.Power_Prosumption + Chassis_Motor3.Power_Prosumption + Chassis_Motor4.Power_Prosumption;
    Chassis_Directive_Motor_Power_Prosumption = Chassis_MotorA.Power_Prosumption + Chassis_MotorB.Power_Prosumption + Chassis_MotorC.Power_Prosumption + Chassis_MotorD.Power_Prosumption;
    Chassis_Gross_Power_Prosumption = Chassis_Traction_Motor_Power_Prosumption + Chassis_Directive_Motor_Power_Prosumption;
    if (Chassis_Gross_Power_Prosumption > max_power) //如果能量消耗大于最大功率
    {
        float Power_Limitation_K;
        //计算功率缩放系数K
        float Gross_Sum_OmegaReal_Times_TauCmd;
        Gross_Sum_OmegaReal_Times_TauCmd = abs(Chassis_Motor1.actual_speed * Chassis_Motor1.Target_Torque) + abs(Chassis_Motor2.actual_speed * Chassis_Motor2.Target_Torque) + abs(Chassis_Motor3.actual_speed * Chassis_Motor3.Target_Torque) + abs(Chassis_Motor4.actual_speed * Chassis_Motor4.Target_Torque) + abs(Chassis_MotorA.actual_speed * Chassis_MotorA.Target_Torque) + abs(Chassis_MotorB.actual_speed * Chassis_MotorB.Target_Torque) + abs(Chassis_MotorC.actual_speed * Chassis_MotorC.Target_Torque) + abs(Chassis_MotorD.actual_speed * Chassis_MotorD.Target_Torque);
        float Gross_Sum_OmegaReal_Times_TauCmd_Square;
        Gross_Sum_OmegaReal_Times_TauCmd_Square = Square(Chassis_Motor1.actual_speed * Chassis_Motor1.Target_Torque) + Square(Chassis_Motor2.actual_speed * Chassis_Motor2.Target_Torque) + Square(Chassis_Motor3.actual_speed * Chassis_Motor3.Target_Torque) + Square(Chassis_Motor4.actual_speed * Chassis_Motor4.Target_Torque) + Square(Chassis_MotorA.actual_speed * Chassis_MotorA.Target_Torque) + Square(Chassis_MotorB.actual_speed * Chassis_MotorB.Target_Torque) + Square(Chassis_MotorC.actual_speed * Chassis_MotorC.Target_Torque) + Square(Chassis_MotorD.actual_speed * Chassis_MotorD.Target_Torque);
        float Gross_Sum_TauCmd_Square;
        Gross_Sum_TauCmd_Square = Square(Chassis_Motor1.Target_Torque) + Square(Chassis_Motor2.Target_Torque) + Square(Chassis_Motor3.Target_Torque) + Square(Chassis_Motor4.Target_Torque) + Square(Chassis_MotorA.Target_Torque) + Square(Chassis_MotorB.Target_Torque) + Square(Chassis_MotorC.Target_Torque) + Square(Chassis_MotorD.Target_Torque);
        float Gross_Sum_OmegaReal_Square;
        Gross_Sum_OmegaReal_Square = Square(Chassis_Motor1.actual_speed) + Square(Chassis_Motor2.actual_speed) + Square(Chassis_Motor3.actual_speed) + Square(Chassis_Motor4.actual_speed) + Square(Chassis_MotorA.actual_speed) + Square(Chassis_MotorB.actual_speed) + Square(Chassis_MotorC.actual_speed) + Square(Chassis_MotorD.actual_speed);
        Power_Limitation_K = -Gross_Sum_OmegaReal_Times_TauCmd + sqrt(Gross_Sum_OmegaReal_Times_TauCmd_Square - 4 * Power_K1 * Gross_Sum_TauCmd_Square * (Power_K2 * Gross_Sum_OmegaReal_Square - max_power)); //分子部分
        Power_Limitation_K = Power_Limitation_K / 2.0f * Power_K1 / Gross_Sum_TauCmd_Square;
        //对电流功率进行限制
        Chassis_Motor1.target_current = Chassis_Motor1.target_current * Power_Limitation_K;
        Chassis_Motor2.target_current = Chassis_Motor2.target_current * Power_Limitation_K;
        Chassis_Motor3.target_current = Chassis_Motor3.target_current * Power_Limitation_K;
        Chassis_Motor4.target_current = Chassis_Motor4.target_current * Power_Limitation_K;
        Chassis_MotorA.target_current = Chassis_MotorA.target_current * Power_Limitation_K;
        Chassis_MotorB.target_current = Chassis_MotorB.target_current * Power_Limitation_K;
        Chassis_MotorC.target_current = Chassis_MotorC.target_current * Power_Limitation_K;
        Chassis_MotorD.target_current = Chassis_MotorD.target_current * Power_Limitation_K;

    }
}

/***********************************************************
*@fuction	:All_Directive_Motor_Angle_Ready
*@brief		:判断转向电机的角度是否已经接近目标值
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-04
***********************************************************/

int All_Directive_Motor_Angle_Ready(void)
{
    if (abs(Chassis_MotorA.Target_Angle - Chassis_MotorA.Total_Angle) < Directive_Motor_Max_Err_Angle)
        if (abs(Chassis_MotorB.Target_Angle - Chassis_MotorB.Total_Angle) < Directive_Motor_Max_Err_Angle)
            if (abs(Chassis_MotorC.Target_Angle - Chassis_MotorC.Total_Angle) < Directive_Motor_Max_Err_Angle)
                if (abs(Chassis_MotorD.Target_Angle - Chassis_MotorD.Total_Angle) < Directive_Motor_Max_Err_Angle)
                    return 1;
    return 0;
}
