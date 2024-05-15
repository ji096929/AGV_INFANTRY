/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-10 14:10:14
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 15:04:22
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\chassis_move\kinematic.c
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "kinematic.h"
#include "math.h"
float Float_Abs(float a)
{
	if (a < 0)
		return -a;
	else
		return a;
}

float Square(float Input) // ÊÊÓÃÓÚ¸¡µãÊýµÄÆ½·½º¯Êý
{
	float Ans;
	Ans = Input * Input;
	return Ans;
}

double My_atan(double y, double x)
{
	double atan;
	atan = atan2(y, x);
	if (x == 0)
	{
		if (y > 0)
			atan = PI / 2;
		else if (y < 0)
			atan = -PI / 2;
	}
	if (y == 0)
	{
		if (x > 0)
			atan = 0;
		else if (x < 0)
			atan = PI;
	}
	return atan;
}

void AGV_Calculate_SPeed(CHASSIS_T *chassis)
{
	float Chassis_Vr;
	Chassis_Vr = chassis->command.vw * (half_width + half_length);

	chassis->A_motor.target_speed.linear_vel = sqrt(Square(chassis->command.vy - Chassis_Vr * cos(PI / 4) + Square(chassis->command.vx - Chassis_Vr * sin(PI / 4))));

	chassis->B_motor.target_speed.linear_vel = sqrt(Square(chassis->command.vy + Chassis_Vr * cos(PI / 4) + Square(chassis->command.vx - Chassis_Vr * sin(PI / 4))));

	chassis->C_motor.target_speed.linear_vel = sqrt(Square(chassis->command.vy + Chassis_Vr * cos(PI / 4) + Square(chassis->command.vx + Chassis_Vr * sin(PI / 4))));

	chassis->D_motor.target_speed.linear_vel = sqrt(Square(chassis->command.vy + Chassis_Vr * cos(PI / 4) + Square(chassis->command.vx - Chassis_Vr * sin(PI / 4))));

	// BD电机安装方向不同
	chassis->A_motor.target_speed.rpm = chassis->A_motor.target_speed.linear_vel * VEL2RPM;
	chassis->B_motor.target_speed.rpm = -chassis->B_motor.target_speed.linear_vel * VEL2RPM;
	chassis->C_motor.target_speed.rpm = -chassis->C_motor.target_speed.linear_vel * VEL2RPM;
	chassis->D_motor.target_speed.rpm = chassis->D_motor.target_speed.linear_vel * VEL2RPM;
}

void AGV_Calc_Angle_Speed(CHASSIS_T *chassis)
{
	float Chassis_Vr_A = chassis->command.vw * R_A;
	float Chassis_Vr_B = chassis->command.vw * R_B;
	float Chassis_Vr_C = chassis->command.vw * R_C;
	float Chassis_Vr_D = chassis->command.vw * R_D;

	float Vy_A = chassis->command.vy - Chassis_Vr_A * cos(THETA_A);
	float Vx_A = chassis->command.vx - Chassis_Vr_A * sin(THETA_A);

	float Vy_B = chassis->command.vy + Chassis_Vr_B * cos(THETA_B);
	float Vx_B = chassis->command.vx - Chassis_Vr_B * sin(THETA_B);

	float Vy_C = chassis->command.vy + Chassis_Vr_C * cos(THETA_C);
	float Vx_C = chassis->command.vx + Chassis_Vr_C * sin(THETA_C);

	float Vy_D = chassis->command.vy - Chassis_Vr_D * cos(THETA_D);
	float Vx_D = chassis->command.vx + Chassis_Vr_D * sin(THETA_D);

	chassis->A_motor.ChassisCoordinate_Angle = My_atan(Vy_A, Vx_A) * RAD_TO_8191;
	chassis->B_motor.ChassisCoordinate_Angle = My_atan(Vy_B, Vx_B) * RAD_TO_8191;
	chassis->C_motor.ChassisCoordinate_Angle = My_atan(Vy_C, Vx_C) * RAD_TO_8191;
	chassis->D_motor.ChassisCoordinate_Angle = My_atan(Vy_D, Vx_D) * RAD_TO_8191;

	chassis->A_motor.target_speed.linear_vel = sqrt(Square(Vy_A) + Square(Vx_A));
	chassis->B_motor.target_speed.linear_vel = sqrt(Square(Vy_B) + Square(Vx_B));
	chassis->C_motor.target_speed.linear_vel = sqrt(Square(Vy_C) + Square(Vx_C));
	chassis->D_motor.target_speed.linear_vel = sqrt(Square(Vy_D) + Square(Vx_D));

	// BD电机安装方向不同
	chassis->A_motor.target_speed.rpm = -chassis->A_motor.target_speed.linear_vel * VEL2RPM;
	chassis->B_motor.target_speed.rpm = -chassis->B_motor.target_speed.linear_vel * VEL2RPM;
	chassis->C_motor.target_speed.rpm = -chassis->C_motor.target_speed.linear_vel * VEL2RPM;
	chassis->D_motor.target_speed.rpm = -chassis->D_motor.target_speed.linear_vel * VEL2RPM;
}

void AGV_Vector_Composition_In_ChassisCoordinate(CHASSIS_T *chassis)
{

	float Radian_Theta, Radian_Conversion_Angle;
	float X_Components, Y_Components, Sine, Cosine;
	float Gimbal_V, Chassis_Vr;
	extern int Chassis_Angle;

	Gimbal_V = sqrt(chassis->command.vx * chassis->command.vx + chassis->command.vy * chassis->command.vy);
	Chassis_Vr = chassis->command.vw * (half_width + half_length);
	Radian_Theta = chassis->parameter.relative_angle / 360.00f * (2.00f * PI);

	Radian_Conversion_Angle = Radian_Theta + atan2(chassis->command.vy, chassis->command.vx);
	while (Radian_Conversion_Angle > PI)
		Radian_Conversion_Angle = Radian_Conversion_Angle - 2 * PI;
	while (Radian_Conversion_Angle < -PI)
		Radian_Conversion_Angle = Radian_Conversion_Angle + 2 * PI;

	Sine = sin(Radian_Conversion_Angle);
	Cosine = cos(Radian_Conversion_Angle);

	if (chassis->command.vy > 0 && chassis->command.vx == 0)
		Sine = 1;
	if (chassis->command.vy < 0 && chassis->command.vx == 0)
		Sine = -1;

	if (Radian_Conversion_Angle > 0)
		Y_Components = Gimbal_V * Float_Abs(Sine) + Chassis_Vr;
	else
		Y_Components = Gimbal_V * (-Float_Abs(Sine)) + Chassis_Vr;

	if (Radian_Conversion_Angle < PI / 2 && Radian_Conversion_Angle > -PI / 2)
		X_Components = Gimbal_V * Float_Abs(Cosine) + Chassis_Vr;
	else
		X_Components = Gimbal_V * (-Float_Abs(Cosine)) + Chassis_Vr;

	chassis->A_motor.ChassisCoordinate_Angle = -atan2(Y_Components, X_Components) / PI / 2 * 8191;
	chassis->A_motor.target_speed.linear_vel = Square(X_Components) + Square(Y_Components);
	chassis->A_motor.target_speed.linear_vel = sqrt(chassis->A_motor.target_speed.linear_vel);

	if (Radian_Conversion_Angle > 0)
		Y_Components = Gimbal_V * Float_Abs(Sine) + Chassis_Vr;
	else
		Y_Components = Gimbal_V * (-Float_Abs(Sine)) + Chassis_Vr;

	if (Radian_Conversion_Angle < PI / 2 && Radian_Conversion_Angle > -PI / 2)
		X_Components = Gimbal_V * Float_Abs(Cosine) - Chassis_Vr;
	else
		X_Components = Gimbal_V * (-Float_Abs(Cosine)) - Chassis_Vr;

	chassis->B_motor.ChassisCoordinate_Angle = -atan2(Y_Components, X_Components) / PI / 2 * 8191;
	chassis->B_motor.target_speed.linear_vel = Square(X_Components) + Square(Y_Components);
	chassis->B_motor.target_speed.linear_vel = sqrt(chassis->B_motor.target_speed.linear_vel);

	if (Radian_Conversion_Angle > 0)
		Y_Components = Gimbal_V * Float_Abs(Sine) - Chassis_Vr;
	else
		Y_Components = Gimbal_V * (-Float_Abs(Sine)) - Chassis_Vr;

	if (Radian_Conversion_Angle < PI / 2 && Radian_Conversion_Angle > -PI / 2)
		X_Components = Gimbal_V * Float_Abs(Cosine) - Chassis_Vr;
	else
		X_Components = Gimbal_V * (-Float_Abs(Cosine)) - Chassis_Vr;

	chassis->C_motor.ChassisCoordinate_Angle = -atan2(Y_Components, X_Components) / PI / 2 * 8191;
	chassis->C_motor.target_speed.linear_vel = Square(X_Components) + Square(Y_Components);
	chassis->C_motor.target_speed.linear_vel = sqrt(chassis->C_motor.target_speed.linear_vel);

	if (Radian_Conversion_Angle > 0)
		Y_Components = Gimbal_V * Float_Abs(Sine) - Chassis_Vr;
	else
		Y_Components = Gimbal_V * (-Float_Abs(Sine)) - Chassis_Vr;

	if (Radian_Conversion_Angle < PI / 2 && Radian_Conversion_Angle > -PI / 2)
		X_Components = Gimbal_V * Float_Abs(Cosine) + Chassis_Vr;
	else
		X_Components = Gimbal_V * (-Float_Abs(Cosine)) + Chassis_Vr;

	chassis->D_motor.ChassisCoordinate_Angle = -atan2(Y_Components, X_Components) / PI / 2 * 8191;
	chassis->D_motor.target_speed.linear_vel = Square(X_Components) + Square(Y_Components);
	chassis->D_motor.target_speed.linear_vel = sqrt(chassis->D_motor.target_speed.linear_vel);

	chassis->A_motor.target_speed.rpm = chassis->A_motor.target_speed.linear_vel * VEL2RPM;
	chassis->B_motor.target_speed.rpm = chassis->B_motor.target_speed.linear_vel * VEL2RPM;
	chassis->C_motor.target_speed.rpm = chassis->C_motor.target_speed.linear_vel * VEL2RPM;
	chassis->D_motor.target_speed.rpm = chassis->D_motor.target_speed.linear_vel * VEL2RPM;
}

void AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate(CHASSIS_T *chassis)
{
	chassis->A_motor.last_target_angle = chassis->A_motor.target_angle;
	chassis->B_motor.last_target_angle = chassis->B_motor.target_angle;
	chassis->C_motor.last_target_angle = chassis->C_motor.target_angle;
	chassis->D_motor.last_target_angle = chassis->D_motor.target_angle;

	//	chassis->A_motor.target_angle = chassis->A_motor.ChassisCoordinate_Angle + chassis->A_motor.zero_position;
	//	chassis->B_motor.target_angle = chassis->B_motor.ChassisCoordinate_Angle + chassis->B_motor.zero_position;
	//	chassis->C_motor.target_angle = chassis->C_motor.ChassisCoordinate_Angle + chassis->C_motor.zero_position;
	//	chassis->D_motor.target_angle = chassis->D_motor.ChassisCoordinate_Angle + chassis->D_motor.zero_position;

	// 轮子方向超前坐标方向90°
	chassis->A_motor.target_angle = chassis->A_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
	chassis->B_motor.target_angle = chassis->B_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
	chassis->C_motor.target_angle = chassis->C_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
	chassis->D_motor.target_angle = chassis->D_motor.ChassisCoordinate_Angle - PI / 2 / (2 * PI) * 8191;
	if (chassis->parameter.break_mode)
	{
		chassis->A_motor.target_angle = 8191 * (315) / 360;
		chassis->B_motor.target_angle = 8191 * (225) / 360;
		chassis->C_motor.target_angle = 8191 * (135) / 360;
		chassis->D_motor.target_angle = 8191 * (45) / 360;
	}
	if (chassis->A_motor.target_angle > 8191)
		chassis->A_motor.target_angle -= 8191;
	if (chassis->A_motor.target_angle < 0)
		chassis->A_motor.target_angle += 8191;
	if (chassis->B_motor.target_angle > 8191)
		chassis->B_motor.target_angle -= 8191;
	if (chassis->B_motor.target_angle < 0)
		chassis->B_motor.target_angle += 8191;
	if (chassis->C_motor.target_angle > 8191)
		chassis->C_motor.target_angle -= 8191;
	if (chassis->C_motor.target_angle < 0)
		chassis->C_motor.target_angle += 8191;
	if (chassis->D_motor.target_angle > 8191)
		chassis->D_motor.target_angle -= 8191;
	if (chassis->D_motor.target_angle < 0)
		chassis->D_motor.target_angle += 8191;

	chassis->A_motor.target_speed.output = (int)(chassis->A_motor.target_speed.rpm) * 1.0f;
	chassis->B_motor.target_speed.output = (int)(chassis->B_motor.target_speed.rpm) * 1.0f;
	chassis->C_motor.target_speed.output = (int)(chassis->C_motor.target_speed.rpm) * 1.0f;
	chassis->D_motor.target_speed.output = (int)(chassis->D_motor.target_speed.rpm) * 1.0f;
}

void Speed_Limitation(CHASSIS_T *chassis)
{
	int temp = 0;
	temp = Float_Abs(chassis->A_motor.target_speed.output);
	if (Float_Abs(chassis->B_motor.target_speed.output) > temp)
		temp = Float_Abs(chassis->B_motor.target_speed.output);
	if (Float_Abs(chassis->C_motor.target_speed.output) > temp)
		temp = Float_Abs(chassis->C_motor.target_speed.output);
	if (Float_Abs(chassis->D_motor.target_speed.output) > temp)
		temp = Float_Abs(chassis->D_motor.target_speed.output);

	if (temp > MAX_MOTOR_SPEED)
	{
		chassis->A_motor.target_speed.output = (int)(chassis->A_motor.target_speed.output * MAX_MOTOR_SPEED * 1.0 / temp);
		chassis->B_motor.target_speed.output = (int)(chassis->B_motor.target_speed.output * MAX_MOTOR_SPEED * 1.0 / temp);
		chassis->C_motor.target_speed.output = (int)(chassis->C_motor.target_speed.output * MAX_MOTOR_SPEED * 1.0 / temp);
		chassis->D_motor.target_speed.output = (int)(chassis->D_motor.target_speed.output * MAX_MOTOR_SPEED * 1.0 / temp);
	}
	else
	{
		chassis->A_motor.target_speed.output = (int)(chassis->A_motor.target_speed.output);
		chassis->B_motor.target_speed.output = (int)(chassis->B_motor.target_speed.output);
		chassis->C_motor.target_speed.output = (int)(chassis->C_motor.target_speed.output);
		chassis->D_motor.target_speed.output = (int)(chassis->D_motor.target_speed.output);
	}
}

float AGV_tan, AGV_ang;
float AGV_DirectiveMotor_RobotMotion_To_TargetStatus(float linear_x, float linear_y, float Theta)
{
	// AGV_ang = -atan2(linear_y,linear_x)/3.1415/2* 8191;// - Theta/360.0f * 8191*10;//±àÂëÆ÷½Ç¶ÈÖµµÄ½Ç¶È
	AGV_ang = -atan2(linear_y, linear_x) / 3.1415;
	return AGV_ang;
}

float chassis_angle;
void Chassis_Speed_Control(CHASSIS_T *chassis)
{
	chassis_angle = AGV_DirectiveMotor_RobotMotion_To_TargetStatus(chassis->command.vx, chassis->command.vx, chassis->parameter.relative_angle);
	if (chassis->command.vx == 0 && chassis->command.vy == 0 && chassis->command.vw == 0)
		chassis->parameter.break_mode = 1;
	else
		chassis->parameter.break_mode = 0;

	// 旧解算
	// AGV_Vector_Composition_In_ChassisCoordinate(chassis);

	// 测试
	// chassis->command.vw = 0;
	// 新解算
	AGV_Calc_Angle_Speed(chassis);

	AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate(chassis);
	Speed_Limitation(chassis);
}