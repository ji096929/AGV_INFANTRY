#include "steering_wheel.h"
#include "steering_communication.h"
#include "chassis_power_control.h"
#include <cmath>
#if defined(STM32F105) | defined(STM32F407)
	#include "can.h"
#endif



/**
 * @brief 初始化舵轮的PID参数。
 * @param steering_wheel 指向 steering_wheel_t 结构的指针。
 */
void Steering_Wheel_PID_HandleInit(steering_wheel_t *steering_wheel)
{
	// 先把PID结构体置零
	memset(&steering_wheel->directive_part.motor.PID_Handles	, 0, sizeof(steering_wheel->directive_part.motor.PID_Handles));
	memset(&steering_wheel->motion_part.motor.PID_Handles		, 0, sizeof(steering_wheel->motion_part.motor.PID_Handles));
	steering_wheel->directive_part.motor.PID_Handles.position_loop_handle.hDefKpGain = 7;
	steering_wheel->directive_part.motor.PID_Handles.position_loop_handle.hKpDivisorPOW2 = 6;
	steering_wheel->directive_part.motor.PID_Handles.position_loop_handle.hDefKdGain = 1;
	steering_wheel->directive_part.motor.PID_Handles.position_loop_handle.hKdDivisorPOW2 = 0;
	steering_wheel->directive_part.motor.PID_Handles.position_loop_handle.hUpperOutputLimit = 2000;
	steering_wheel->directive_part.motor.PID_Handles.position_loop_handle.hLowerOutputLimit = -2000;
	
	// 转向电机速度环 默认Kp Ki Kd 写入
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.hDefKpGain = 400;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.hKpDivisorPOW2 = 1;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.hDefKiGain = 5;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.hKiDivisorPOW2 = 2;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.hUpperOutputLimit = 6000;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.hLowerOutputLimit = -6000;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.wUpperIntegralLimit = 1600;
	steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle.wLowerIntegralLimit = -1600;
	// 动力电机速度环 默认Kp Ki Kd 写入
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hDefKpGain = 72;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hKpDivisorPOW2 = 2;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hDefKiGain = 20;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hKiDivisorPOW2 = 2;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hDefKdGain = 0;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hKdDivisorPOW2 =0;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hUpperOutputLimit = 16000;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.hLowerOutputLimit = -16000;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.wUpperIntegralLimit = 1000;
	steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle.wLowerIntegralLimit = -1000;
	// 调用PID结构体初始化函数，数据写入PID结构体中
	PID_HandleInit(&steering_wheel->directive_part.motor.PID_Handles.position_loop_handle);
	PID_HandleInit(&steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle);
	PID_HandleInit(&steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle);
}

/**
 * @brief 初始化舵轮列表，此列表可以用于已知舵轮的某些参数 （如 CAN ID） 情况下返回舵轮的句柄。
 * @param steering_wheel 舵轮的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
steering_handle_list_t steering_handle_list[MAXIMUM_STEERING_HANDLE_NUM];
STEERING_WHEEL_RETURN_T Steering_HandleListAdd(steering_wheel_t *steering)
{
	for (int i=0; i<MAXIMUM_STEERING_HANDLE_NUM; i++) // 从表头开始找有没有空位
		if (steering_handle_list[i].handle == 0)
		{
			steering_handle_list[i].handle	= steering;
			steering_handle_list[i].CANID	= steering->parameter.CANID;
			return STEERING_WHEEL_OK;
		}
	return STEERING_WHEEL_ERROR;
}
/**
 * @brief 在舵轮列表中确认是否有某个舵轮的句柄，确保句柄合法性
 * @param steering_wheel 舵轮的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
STEERING_WHEEL_RETURN_T Steering_CheckHandleLegitimacy(steering_wheel_t *steering)
{
	for (int i=0; i<MAXIMUM_STEERING_HANDLE_NUM; i++) // 从表头开始找
		if (steering_handle_list[i].handle == steering)
			return STEERING_WHEEL_OK;
		return STEERING_WHEEL_ILLEGAL_HANDLE; // 句柄找不到，不合法
}
/**
 * @brief 初始化舵轮组件，包括电机、齿轮和编码器。
 * @param steering_wheel 舵轮的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
STEERING_WHEEL_RETURN_T Steering_Wheel_HandleInit(steering_wheel_t *steering_wheel)
{
	// 初始化舵轮列表
	memset(&steering_handle_list, NULL, sizeof(steering_handle_list));
	
	// 初始化PID结构体
	Steering_Wheel_PID_HandleInit(steering_wheel);
	#if defined(DIRECTIVE_MOTOR_M3508) | defined(MOTION_MOTOR_M3508)
		// 转向电机及其齿轮初始化
		M3508_gear_parameter_t direction_motor_gear_init;
		// 舵轮的传动齿轮是广义上的减速箱，所以给了减速比 (doge
		#ifdef AGV_HERO
		direction_motor_gear_init.reduction_rate	= 10;
		direction_motor_gear_init.bus				= &M3508_bus_1;
		direction_motor_gear_init.ESC_ID			= 1;
		direction_motor_gear_init.resistance_torque	= 0;
		direction_motor_gear_init.handle			= &hcan1;
		#endif
		#ifdef AGV_STANDARD
		direction_motor_gear_init.reduction_rate	= 8;
		direction_motor_gear_init.bus				= &M3508_bus_1;
		direction_motor_gear_init.ESC_ID			= 1;
		direction_motor_gear_init.resistance_torque	= 0;
		direction_motor_gear_init.handle			= &hcan1;
		#endif
		
		M3508_gear_parameter_init(&steering_wheel->directive_part.motor.M3508_kit, &direction_motor_gear_init);
		// 动力电机及其齿轮初始化
		M3508_gear_parameter_t motion_motor_gear_init;
			#ifdef AGV_HERO
		motion_motor_gear_init.reduction_rate		= 3591/187.f;
		motion_motor_gear_init.bus					= &M3508_bus_1;
		motion_motor_gear_init.ESC_ID				= 2;
		motion_motor_gear_init.resistance_torque	= 0;
		motion_motor_gear_init.handle			= &hcan1;
		#endif
		#ifdef AGV_STANDARD
		motion_motor_gear_init.reduction_rate		= 14;
		motion_motor_gear_init.bus					= &M3508_bus_1;
		motion_motor_gear_init.ESC_ID				= 2;
		motion_motor_gear_init.resistance_torque	= 0;
		motion_motor_gear_init.handle			= &hcan1;
		#endif
		M3508_gear_parameter_init(&steering_wheel->motion_part.motor.M3508_kit, &motion_motor_gear_init);
	#endif
		#ifdef AGV_BOARD_A
		steering_wheel->command.protocol_position = A_ENCODER_ZERO_POSION;
		#endif
		#ifdef AGV_BOARD_B
		steering_wheel->command.protocol_position = B_ENCODER_ZERO_POSION;
		#endif
		#ifdef AGV_BOARD_C
		steering_wheel->command.protocol_position = C_ENCODER_ZERO_POSION;
		#endif
		#ifdef AGV_BOARD_D
		steering_wheel->command.protocol_position = D_ENCODER_ZERO_POSION;
		#endif
	
	#if defined(DIRECTIVE_ENCODER_BRITER_ENCODER)
		// 编码器初始化
		briter_encoder_Init(&steering_wheel->directive_part.encoder.briter_encoder);
		birter_encoder_parameter_t birter_encoder_init;
		birter_encoder_init.baud_rate				= BRITER_ENCODER_SET_CAN_BAUD_RATE_1M;
		birter_encoder_init.call_back_mode			= BRITER_ENCODER_SET_CALLBACK_REQUEST;
		birter_encoder_init.increment_direction		= BRITER_ENCODER_INCREMENT_DIRECTION_CW;
		#ifdef AGV_BOARD_A
		birter_encoder_init.CAN_ID					= A_ENCODER_ID;
		#endif
		#ifdef AGV_BOARD_B
		birter_encoder_init.CAN_ID					= B_ENCODER_ID;
		#endif
		#ifdef AGV_BOARD_C
		birter_encoder_init.CAN_ID					= C_ENCODER_ID;
		#endif
		#ifdef AGV_BOARD_D
		birter_encoder_init.CAN_ID					= D_ENCODER_ID;
		#endif
		briter_encoder_parameter_init(&steering_wheel->directive_part.encoder.briter_encoder, &birter_encoder_init);
		#ifdef AGV_HERO
		steering_wheel->directive_part.encoder.parameter.lsbs_per_encoder_round			= 1024;
		steering_wheel->directive_part.encoder.parameter.encoder_rounds_per_part_round	= 5;
		steering_wheel->directive_part.encoder.parameter.lsbs_per_part_round = steering_wheel->directive_part.encoder.parameter.lsbs_per_encoder_round*steering_wheel->directive_part.encoder.parameter.encoder_rounds_per_part_round;
		#endif
		#ifdef AGV_STANDARD
		steering_wheel->directive_part.encoder.parameter.lsbs_per_encoder_round			= 1024;
		steering_wheel->directive_part.encoder.parameter.encoder_rounds_per_part_round	= 4;
		steering_wheel->directive_part.encoder.parameter.lsbs_per_part_round = steering_wheel->directive_part.encoder.parameter.lsbs_per_encoder_round*steering_wheel->directive_part.encoder.parameter.encoder_rounds_per_part_round;
		#endif
		
		#endif
	
	// 初始化舵轮 CAN ID
	
			#ifdef AGV_BOARD_A
		steering_wheel->parameter.CANID = A_STEERING_CAN_ID;
		#endif
		#ifdef AGV_BOARD_B
		steering_wheel->parameter.CANID = B_STEERING_CAN_ID;
		#endif
		#ifdef AGV_BOARD_C
		steering_wheel->parameter.CANID = C_STEERING_CAN_ID;
		#endif
		#ifdef AGV_BOARD_D
		steering_wheel->parameter.CANID = D_STEERING_CAN_ID;
		#endif
	steering_wheel->parameter.invert_flag = 1;//角度优化所用
	steering_wheel->parameter.enable = STEERING_WHEEL_ACTIVATED;
	if (Steering_HandleListAdd(steering_wheel))
		return STEERING_WHEEL_ERROR;
	return STEERING_WHEEL_OK;
}

steering_wheel_t *Steering_FindSteeringHandle_via_CANID(uint8_t CANID)
{
	for (int i=0; i<MAXIMUM_STEERING_HANDLE_NUM; i++)
		if (steering_handle_list[i].CANID == CANID)
			return steering_handle_list[i].handle;
	return STEERING_ILLEGAL_HANDLE;
}

/**
 * @brief 更新电机和编码器的原始反馈信息。
 * @param steering_wheel 舵轮的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
STEERING_WHEEL_RETURN_T Steering_Wheel_MotorAndEncoderFeedbackUpdate(steering_wheel_t *steering_wheel)
{
	// 获取编码器多圈角度
    #if defined(DIRECTIVE_ENCODER_BRITER_ENCODER)
		// 获取多圈角度值
		steering_wheel->directive_part.encoder.feedback.position = steering_wheel->directive_part.encoder.briter_encoder.status.total_angle;
		// 计算角速度
		#if defined(STM32F105)
			steering_wheel->directive_part.encoder.feedback.get_tick = total_count;
		#endif
		uint32_t dt, dw;
	
		dt = steering_wheel->directive_part.encoder.feedback.get_tick - steering_wheel->directive_part.encoder.feedback.last_get_tick;
		dw = steering_wheel->directive_part.encoder.briter_encoder.status.total_angle - steering_wheel->directive_part.encoder.briter_encoder.status.last_total_angle;

		steering_wheel->directive_part.encoder.feedback.speed = dw / dt;
		steering_wheel->directive_part.encoder.feedback.speed = steering_wheel->directive_part.motor.M3508_kit.status.output_speed_rpm; // 这里测试时用的3508电机的角速度，懒得改回去了
		briter_encoder_request_tatal_angle(&steering_wheel->directive_part.encoder.briter_encoder);
		steering_wheel->directive_part.encoder.feedback.last_get_tick = steering_wheel->directive_part.encoder.feedback.get_tick;
	#endif
	// 获取转向电机经过齿轮传动后的角度、速度、力矩
	#if defined(DIRECTIVE_MOTOR_M3508)
		steering_wheel->directive_part.motor.feedback.speed		= steering_wheel->directive_part.motor.M3508_kit.status.output_speed_rpm;
		steering_wheel->directive_part.motor.feedback.torque	= steering_wheel->directive_part.motor.M3508_kit.status.output_torque;
	#endif
	// 获取动力电机经过齿轮传动后的角度、速度、力矩
	#if defined(DIRECTIVE_MOTOR_M3508)
		steering_wheel->motion_part.motor.feedback.position	= steering_wheel->motion_part.motor.M3508_kit.status.output_angle_lsb;
		steering_wheel->motion_part.motor.feedback.speed	= steering_wheel->motion_part.motor.M3508_kit.status.output_speed_rpm;
		steering_wheel->motion_part.motor.feedback.torque	= steering_wheel->motion_part.motor.M3508_kit.status.output_torque;
	#endif
	return STEERING_WHEEL_OK;
}

/**
* @brief 把原始的编码器或电机信息转化成符合协议的位置信息。
 * @param steering_wheel 舵轮的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
STEERING_WHEEL_RETURN_T Steering_Wheel_MotorAndEncoderStatusUpdate(steering_wheel_t *steering_wheel)
{
	#if defined(DIRECTIVE_WITH_ENCODER)
		// 多圈角度（编码器50圈，对应舵5圈）变为单圈角度（编码器10圈，对应舵1圈）
		steering_wheel->directive_part.encoder.status.position	= steering_wheel->directive_part.encoder.feedback.position % steering_wheel->directive_part.encoder.parameter.lsbs_per_part_round;
		// 由于齿轮传动使得编码器转动方向为CW时，舵转动方向为CCW，反之亦然。所以要对称处理
		if (steering_wheel->directive_part.encoder.parameter.encoder_directive_part_direction == DIRECTION_INVERSE)
			steering_wheel->directive_part.encoder.status.position	= steering_wheel->directive_part.encoder.parameter.lsbs_per_part_round - steering_wheel->directive_part.encoder.status.position;
		// 多圈角度映射到 统一的角度分辨率（也是通信传输时的分辨率）
		steering_wheel->directive_part.encoder.status.position	= steering_wheel->directive_part.encoder.status.position * PROTOCOL_POSITION_LSBS / steering_wheel->directive_part.encoder.parameter.lsbs_per_part_round;
		steering_wheel->directive_part.encoder.status.speed		= steering_wheel->directive_part.encoder.feedback.speed;
	#else
		steering_wheel->directive_part.motor.status.position	= steering_wheel->directive_part.motor.feedback.position;
		steering_wheel->directive_part.motor.status.speed		= steering_wheel->directive_part.motor.feedback.speed;
		steering_wheel->directive_part.motor.status.torque		= steering_wheel->directive_part.motor.feedback.torque;
	#endif

	steering_wheel->motion_part.motor.status.position	= steering_wheel->motion_part.motor.feedback.position;
	steering_wheel->motion_part.motor.status.speed		= steering_wheel->motion_part.motor.feedback.speed;
	steering_wheel->motion_part.motor.status.torque		= steering_wheel->motion_part.motor.feedback.torque;
	return STEERING_WHEEL_OK;
}

STEERING_WHEEL_RETURN_T Steering_Wheel_PartStatusUpdate(steering_wheel_t *steering_wheel)
{
	#if defined(DIRECTIVE_ENCODER_BRITER_ENCODER)
		steering_wheel->directive_part.status.protocol_position = steering_wheel->directive_part.encoder.status.position;
		steering_wheel->directive_part.status.protocol_speed	= steering_wheel->directive_part.encoder.status.speed;
	#endif
	#if defined(MOTION_MOTOR_M3508)
		steering_wheel->motion_part.status.protocol_speed		= steering_wheel->motion_part.motor.status.speed;
	#endif
	return STEERING_WHEEL_OK;
}

STEERING_WHEEL_RETURN_T Steering_Wheel_StatusUpdate(steering_wheel_t *steering_wheel)
{
	if (Steering_CheckHandleLegitimacy(steering_wheel) == STEERING_WHEEL_OK) 
	{
		Steering_Wheel_MotorAndEncoderFeedbackUpdate(steering_wheel);
		Steering_Wheel_MotorAndEncoderStatusUpdate(steering_wheel);
		Steering_Wheel_PartStatusUpdate(steering_wheel);

		return STEERING_WHEEL_OK;
	}
	else return STEERING_WHEEL_ILLEGAL_HANDLE;
}

/**
 * @brief 根据PID控制和协议命令更新电机的指令。
 * @param steering_wheel 舵轮的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
int16_t testt;
STEERING_WHEEL_RETURN_T Steering_Wheel_MotorCommandUpdate(steering_wheel_t *steering_wheel)
{
	if (Steering_CheckHandleLegitimacy(steering_wheel) == STEERING_WHEEL_OK) 
	{
		int32_t temp_err;
		// 动力电机速度环PID
		temp_err = steering_wheel->motion_part.command.protocol_speed*steering_wheel->parameter.invert_flag - steering_wheel->motion_part.status.protocol_speed;
		steering_wheel->motion_part.motor.command.torque = PID_Controller(&steering_wheel->motion_part.motor.PID_Handles.velocity_loop_handle, temp_err);
		#if defined(MOTION_MOTOR_M3508)
		calculate_torque_current_according_to_scaled_power(chassis_power_control.scaled_power_32);
		steering_wheel->motion_part.motor.M3508_kit.command.torque = steering_wheel->motion_part.motor.command.torque;
#endif
		// 转向电机角度环PID
		
		
		//开启角度优化模式
		if(steering_wheel->parameter.deg_optimization==ENABLE_MINOR_DEG_OPTIMIZEATION)
		{
			int32_t temp_min;
			temp_err = steering_wheel->directive_part.command.protocol_position - steering_wheel->directive_part.status.protocol_position - ((steering_wheel->parameter.invert_flag-1)/2.0f)*HALF_PROTOCOL_POSITION_LSBS;
			if (abs(temp_err)<PROTOCOL_POSITION_LSBS-abs(temp_err))
			temp_min = abs(temp_err);
			else
			temp_min = PROTOCOL_POSITION_LSBS-abs(temp_err);
			if (temp_min > HALF_PROTOCOL_POSITION_LSBS*0.8f)
			steering_wheel->parameter.invert_flag=-steering_wheel->parameter.invert_flag;
			temp_err = steering_wheel->directive_part.command.protocol_position - steering_wheel->directive_part.status.protocol_position + ((steering_wheel->parameter.invert_flag-1)/2.0f)*HALF_PROTOCOL_POSITION_LSBS;
		}
		//关闭角度优化模式
		else
		{
			temp_err = steering_wheel->directive_part.command.protocol_position - steering_wheel->directive_part.status.protocol_position;
		}


		// 开启优劣弧优化模式
		if (steering_wheel->parameter.arc_optimization == ENABLE_MINOR_ARC_OPTIMIZEATION)
		{
			if (temp_err > PROTOCOL_POSITION_LSBS - temp_err)
				temp_err = -(PROTOCOL_POSITION_LSBS - temp_err);
			else if (temp_err < -(PROTOCOL_POSITION_LSBS + temp_err))
				temp_err = (PROTOCOL_POSITION_LSBS + temp_err);
			
		}
		// 关闭优劣弧优化模式
		else 
		{
			//if ()
		}
		//更改
		steering_wheel->directive_part.command.protocol_speed	= PID_Controller(&steering_wheel->directive_part.motor.PID_Handles.position_loop_handle, temp_err);
//		steering_wheel->directive_part.command.protocol_speed	= testt;
		temp_err = steering_wheel->directive_part.command.protocol_speed - steering_wheel->directive_part.status.protocol_speed;
		steering_wheel->directive_part.motor.command.torque	= PID_Controller(&steering_wheel->directive_part.motor.PID_Handles.velocity_loop_handle, temp_err);
		// 由于齿轮传动使得编码器转动方向为CW时，舵转动方向为CCW，反之亦然。所以要对称处理
		if (steering_wheel->directive_part.encoder.parameter.encoder_directive_part_direction == DIRECTION_INVERSE)
			steering_wheel->directive_part.motor.command.torque = steering_wheel->directive_part.motor.command.torque;
		#if defined(DIRECTIVE_MOTOR_M3508)
			steering_wheel->directive_part.motor.M3508_kit.command.torque = steering_wheel->directive_part.motor.command.torque;
		#endif

		return STEERING_WHEEL_OK;
	}
	else return STEERING_WHEEL_ILLEGAL_HANDLE;
}

/*
	将舵轮整体的指令转化为舵轮各个部分的指令
*/
STEERING_WHEEL_RETURN_T Steering_Wheel_PartCommandUpdate(steering_wheel_t *steering_wheel)
{
	steering_wheel->directive_part.command.protocol_position	= steering_wheel->command.protocol_position;
	steering_wheel->motion_part.command.protocol_speed			= steering_wheel->command.protocol_speed;
	return STEERING_WHEEL_OK;
}

STEERING_WHEEL_RETURN_T Steering_Wheel_CommandUpdate(steering_wheel_t *steering_wheel)
{
	Steering_Wheel_PartCommandUpdate(steering_wheel);
	Steering_Wheel_MotorCommandUpdate(steering_wheel);
	return STEERING_WHEEL_OK;
}

STEERING_WHEEL_RETURN_T Steering_Wheel_CommandTransmit(steering_wheel_t *steering_wheel)
{
	if (Steering_CheckHandleLegitimacy(steering_wheel) == STEERING_WHEEL_OK) 
	{
		#if defined(DIRECTIVE_MOTOR_M3508)
			M3508_gear_set_torque_current_lsb(&steering_wheel->directive_part.motor.M3508_kit, steering_wheel->directive_part.motor.M3508_kit.command.torque, SEND_COMMAND_NOW);
		#endif
		#if defined(MOTION_MOTOR_M3508)
		if(chassis_power_control.motor_control_flag)
			{
				M3508_gear_set_torque_current_lsb(&steering_wheel->motion_part.motor.M3508_kit, steering_wheel->motion_part.motor.M3508_kit.command.torque, SEND_COMMAND_NOW);
			}
		#endif
		return STEERING_WHEEL_OK;
	}
	else return STEERING_WHEEL_ILLEGAL_HANDLE;
}

STEERING_WHEEL_RETURN_T Steering_Wheel_SetProtocolPosition(steering_wheel_t *steering_wheel, uint16_t protocol_position)
{
	steering_wheel->command.protocol_position = protocol_position;
	return STEERING_WHEEL_OK;
}
STEERING_WHEEL_RETURN_T Steering_Wheel_SetProtocolSpeed(steering_wheel_t *steering_wheel, uint16_t protocol_speed)
{
	steering_wheel->command.protocol_speed = protocol_speed;
	return STEERING_WHEEL_OK;
}
STEERING_WHEEL_RETURN_T Steering_Wheel_SetProtocolDegMode(steering_wheel_t *steering_wheel, uint8_t deg_mode)
{
	switch(deg_mode)
	{
		case ENABLE_OPTIMIZATION_STATE:
		steering_wheel->parameter.deg_optimization=ENABLE_MINOR_DEG_OPTIMIZEATION;
		break;
		case DISABLE_OPTIMIZATION_STATE :
		steering_wheel->parameter.deg_optimization=DISABLE_MINOR_DEG_OPTIMIZEATION;
		break;
		default:
		break;
	}	
	return STEERING_WHEEL_OK;
}
STEERING_WHEEL_RETURN_T Steering_Wheel_SetProtocolArcMode(steering_wheel_t *steering_wheel, uint8_t arc_mode)
{
	switch(arc_mode)
	{
		case ENABLE_OPTIMIZATION_STATE:
		steering_wheel->parameter.arc_optimization=ENABLE_MINOR_ARC_OPTIMIZEATION;
		break;
		case DISABLE_OPTIMIZATION_STATE :
		steering_wheel->parameter.arc_optimization=DISABLE_MINOR_ARC_OPTIMIZEATION;
		break;
		default:
		break;
	}
	return STEERING_WHEEL_OK;
}
