#include "steering_communication.h"
#include "steering_wheel.h"
#include "chassis_power_control.h"
#if defined(STM32F105) | (STM32F407)
    #include "can.h"
#endif

/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
    CAN_TxHeaderTypeDef steering_communication_CAN_TxHeaderStruct;
    uint32_t  steering_communication_pTxMailbox;
#endif

// Function prototypes
static uint8_t platform_trans(void *handle, uint32_t Ext_CANID, uint8_t aData[]);
static uint8_t platform_get_msg	(void *handle, uint32_t Ext_CANID, uint8_t aData[]);
void steering_communication_tx_queue_pop(steering_communication_ctx_t *ctx);	// 发送环形发送队列中的数据
void steering_communication_tx_queue_push(steering_communication_pack_t pack);	// 将待发送数据送入环形发送队列中
uint64_t steering_communication_GET_PID_PARAMETER_term_handler(PID_Handle_t pid_handle, uint8_t term_offset_id);
steering_communication_pack_t steering_communication_GET_PID_PARAMETER_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack);
void steering_communication_SET_PID_PARAMETER_term_handler(PID_Handle_t *pid_handle, int16_t *rx_byte, uint8_t term_offset_id);
steering_communication_pack_t steering_communication_SET_PID_PARAMETER_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack);


steering_communication_ctx_t steering_communication_ctx;




/*
	用于返回查询某个舵轮的指定 PID 的某环节值
*/
uint64_t steering_communication_GET_PID_PARAMETER_term_handler(PID_Handle_t pid_handle, uint8_t term_offset_id)
{
	uint64_t return_data1 = 0;
	int16_t shortern_UpperIntegralLimit, shortern_LowerIntegralLimit;
	shortern_UpperIntegralLimit = pid_handle.wUpperIntegralLimit; // 原来是32位的，要把它强制换成16位
	shortern_LowerIntegralLimit = pid_handle.wLowerIntegralLimit; // 原来是32位的，要把它强制换成16位
	switch (term_offset_id)
	{
		case OFFSET_OUTPUT_TERM:
			memcpy((uint16_t *)&return_data1,	&pid_handle.hUpperOutputLimit, sizeof(pid_handle.hUpperOutputLimit));
			memcpy((uint16_t *)&return_data1+1,	&pid_handle.hLowerOutputLimit, sizeof(pid_handle.hLowerOutputLimit));
			break;
		case OFFSET_PROPOSITIONAL_TERM:
			memcpy((uint16_t *)&return_data1,	&pid_handle.hKpGain,		sizeof(pid_handle.hKpGain));
			memcpy((uint16_t *)&return_data1+1,	&pid_handle.hKpDivisorPOW2, sizeof(pid_handle.hKpDivisorPOW2));
			break;
		case OFFSET_INTEGRAL_TERM:
			memcpy((uint16_t *)&return_data1,	&pid_handle.hKiGain,			sizeof(pid_handle.hKiGain));
			memcpy((uint16_t *)&return_data1+1,	&pid_handle.hKiDivisorPOW2,		sizeof(pid_handle.hKiDivisorPOW2));
			memcpy((uint16_t *)&return_data1+2,	&shortern_UpperIntegralLimit,	sizeof(shortern_UpperIntegralLimit));
			memcpy((uint16_t *)&return_data1+3,	&shortern_LowerIntegralLimit,	sizeof(shortern_LowerIntegralLimit));
			break;
		case OFFSET_DIFFERENTIAL_TERM:
			memcpy((uint16_t *)&return_data1,	&pid_handle.hKdGain,		sizeof(pid_handle.hKdGain));
			memcpy((uint16_t *)&return_data1+1,	&pid_handle.hKdDivisorPOW2, sizeof(pid_handle.hKdDivisorPOW2));
			break;
		default:
			memset(&return_data1, 0, sizeof(return_data1));
			break;
	
	}
	return return_data1;
}
/*
	用于返回查询某个舵轮的指定某 PID 值
*/
steering_communication_pack_t steering_communication_GET_PID_PARAMETER_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	uint8_t term_offset_id, loop_offset_id;
	memcpy(&loop_offset_id, &rx_pack.data2,		1);
	memcpy(&term_offset_id, (uint8_t *)&rx_pack.data2+1,	1);
	uint64_t processed_data1;
	switch (loop_offset_id)
	{
		case OFFSET_DIRECTIVE_POSITION_LOOP:
			processed_data1 = steering_communication_GET_PID_PARAMETER_term_handler(steering->directive_part.motor.PID_Handles.position_loop_handle, term_offset_id);
			break;
		case OFFSET_DIRECTIVE_VELOCITY_LOOP:
			processed_data1 = steering_communication_GET_PID_PARAMETER_term_handler(steering->directive_part.motor.PID_Handles.velocity_loop_handle, term_offset_id);
			break;
		case OFFSET_MOTION_POSITION_LOOP:
			processed_data1 = steering_communication_GET_PID_PARAMETER_term_handler(steering->motion_part.motor.PID_Handles.position_loop_handle, term_offset_id);
			break;
		case OFFSET_MOTION_VELOCITY_LOOP:
			processed_data1 = steering_communication_GET_PID_PARAMETER_term_handler(steering->motion_part.motor.PID_Handles.velocity_loop_handle, term_offset_id);
			break;
		default:
			break;
	}
	memcpy(&rx_pack.data1, &processed_data1, sizeof(processed_data1)); // 除了数据区1的内容，其他原封不动
	// 添加返回标志位
	rx_pack.cmd_id = RETURN_CMD_ID;
	return rx_pack;
}


/*
	用于设置某个舵轮的指定 PID 的某环节值
*/
void steering_communication_SET_PID_PARAMETER_term_handler(PID_Handle_t *pid_handle, int16_t *rx_byte, uint8_t term_offset_id)
{
	uint64_t return_data1 = 0;
	switch (term_offset_id)
	{
		case OFFSET_OUTPUT_TERM:
			memcpy(&pid_handle->hUpperOutputLimit,	rx_byte,	sizeof(pid_handle->hUpperOutputLimit));
			memcpy(&pid_handle->hLowerOutputLimit,	rx_byte+1,	sizeof(pid_handle->hLowerOutputLimit));
			break;
		case OFFSET_PROPOSITIONAL_TERM:
			memcpy(&pid_handle->hKpGain,		rx_byte,	sizeof(pid_handle->hKpGain));
			memcpy(&pid_handle->hKpDivisorPOW2,	rx_byte+1,	sizeof(pid_handle->hKpDivisorPOW2));
			break;
		case OFFSET_INTEGRAL_TERM:
			memcpy(&pid_handle->hKiGain,				rx_byte,	sizeof(pid_handle->hKiGain));
			memcpy(&pid_handle->hKiDivisorPOW2,			rx_byte+1,	sizeof(pid_handle->hKiDivisorPOW2));
			memcpy(&pid_handle->wUpperIntegralLimit,	rx_byte+2,	sizeof(int16_t));
			memcpy(&pid_handle->wLowerIntegralLimit,	rx_byte+3,	sizeof(int16_t));
			break;
		case OFFSET_DIFFERENTIAL_TERM:
			memcpy(&pid_handle->hKdGain,		rx_byte,	sizeof(pid_handle->hKdGain));
			memcpy(&pid_handle->hKdDivisorPOW2,	rx_byte+1,	sizeof(pid_handle->hKdDivisorPOW2));
			break;
		default:
			memset(&return_data1, 0, sizeof(return_data1));
			break;
	}
}
/*
	用于设置某个舵轮的指定某 PID 值
*/
steering_communication_pack_t steering_communication_SET_PID_PARAMETER_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	uint8_t term_offset_id, loop_offset_id;
	memcpy(&loop_offset_id, (uint8_t *)&rx_pack.data2,		1);
	memcpy(&term_offset_id, (uint8_t *)&rx_pack.data2+1,	1);
	int16_t rx_data1_byte[4];
	memcpy(&rx_data1_byte, &rx_pack.data1, sizeof(rx_data1_byte)); 
	switch (loop_offset_id)
	{
		case OFFSET_DIRECTIVE_POSITION_LOOP:
			steering_communication_SET_PID_PARAMETER_term_handler(&steering->directive_part.motor.PID_Handles.position_loop_handle, rx_data1_byte, term_offset_id);
			break;
		case OFFSET_DIRECTIVE_VELOCITY_LOOP:
			steering_communication_SET_PID_PARAMETER_term_handler(&steering->directive_part.motor.PID_Handles.velocity_loop_handle, rx_data1_byte, term_offset_id);
			break;
		case OFFSET_MOTION_POSITION_LOOP:
			steering_communication_SET_PID_PARAMETER_term_handler(&steering->motion_part.motor.PID_Handles.position_loop_handle, rx_data1_byte, term_offset_id);
			break;
		case OFFSET_MOTION_VELOCITY_LOOP:
			steering_communication_SET_PID_PARAMETER_term_handler(&steering->motion_part.motor.PID_Handles.velocity_loop_handle, rx_data1_byte, term_offset_id);
			break;
		default:
			break;
	}
	// 添加返回标志位
	rx_pack.cmd_id = RETURN_CMD_ID;
	return rx_pack;
}

/*
	用于初始化订阅列表
*/
steering_communication_subscribe_list_unit_t steering_communication_subscribe_list[SUBSCRIBE_LIST_MAXIMUM_LENGTH];
STEERING_COMMUNICATION_RETURN_T Steering_Communication_SubscribeList_Init(void)
{
	memset(&steering_communication_subscribe_list, 0, sizeof(steering_communication_subscribe_list));
	return STEERING_COMMUNICATION_OK;
}
/*
	用于检查订阅列表是否有空位。如果有空位返回空位的位置，否则返回列表长度
*/
uint8_t steering_communication_SubscirbeList_CheckAvailable(steering_communication_subscribe_list_unit_t list[])
{
	for (int i=0; i<SUBSCRIBE_LIST_MAXIMUM_LENGTH; i++)
		if (list[i].param.content_id == 0)
			return i;
	return SUBSCRIBE_LIST_MAXIMUM_LENGTH;
}
/*
	用于检查订阅列表与当前尝试订阅消息是否有重复。如有有返回重复的位数，否则返回列表长度
*/
uint8_t steering_communication_SubscirbeList_CheckRepeatability(steering_communication_subscribe_list_unit_t list[], uint8_t content_id)
{
	for (int i=0; i<SUBSCRIBE_LIST_MAXIMUM_LENGTH; i++)
		if (list[i].param.content_id == content_id)
			return STEERING_COMMUNICATION_ERROR;
	return SUBSCRIBE_LIST_MAXIMUM_LENGTH;
}

/*
	用于尝试将某一待订阅内容插入订阅消息列表
*/
STEERING_COMMUNICATION_RETURN_T steering_communication_SubscribeList_TryIncert(steering_communication_subscribe_list_unit_t *list, subscribe_param_t param)
{
	// 找一下有没有空位
	uint8_t try_incert1 = steering_communication_SubscirbeList_CheckAvailable(list);
	if (try_incert1 != SUBSCRIBE_LIST_MAXIMUM_LENGTH) // 有空位
	{
		uint8_t try_incert2 = steering_communication_SubscirbeList_CheckRepeatability(list, param.content_id); // 看看有没有订阅过
		if (try_incert2 != SUBSCRIBE_LIST_MAXIMUM_LENGTH) // 有订阅过
		{
			// 覆盖原有内容
			memcpy(&list[try_incert2].param, &param, sizeof(param));
			memcpy(&list[try_incert2].remained_ticks, &param.subscribe_times, 2);
			if (!param.subscribe_times)
				list[try_incert2].subscribe_times_infinite_flag = 1;
			return STEERING_COMMUNICATION_SUBCRIBE_LIST_OVERWRITED;
		}
		else // 没订阅过
		{
			memcpy(&list[try_incert1].param, &param, sizeof(param));
			memcpy(&list[try_incert1].remained_ticks, &param.subscribe_times, 2);
			if (!param.subscribe_times)
				list[try_incert1].subscribe_times_infinite_flag = 1;
			return STEERING_COMMUNICATION_OK;
		}
	}
	else return STEERING_COMMUNICATION_SUBCRIBE_LIST_FULL;
}

/*
	用于提取 SUBSCRIBE_CONTENT_X_OFFSET_ID 对应的取值地址
*/
void *steering_communication_Subscribe_GetValueAdd(steering_wheel_t *steering, uint8_t code)
{
	switch (code)
	{
		case OFFSET_MOTION_PART_RPM:
			return &steering->motion_part.status.protocol_speed;
			break;
		case OFFSET_DIRECTIVE_POSITION_LSB:
			return &steering->directive_part.status.protocol_position;
			break;
		case OFFSET_MOTION_MOTOR_TORQUE_CURRENT_LSB:
			return &steering->motion_part.motor.M3508_kit.status.rotor_torque;
			break;
		case OFFSET_DIRECTIVE_MOTOR_TORQUE_CURRENT_LSB:
			return &steering->directive_part.motor.M3508_kit.status.rotor_torque;
			break;
		default:
			return NULL;
			break;
	}
}

/*
	用于处理 cmd_id 为 ADD_SUBSCRIBE_VALUE 的情况
*/
steering_communication_pack_t steering_communication_ADD_SUBSCRIBE_VALUE_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	// 提取出需要订阅的值
	uint8_t content_2_offset_id, content_1_offset_id;
	memcpy(&content_1_offset_id, (uint8_t *)&rx_pack.data2,		1);
	memcpy(&content_2_offset_id, (uint8_t *)&rx_pack.data2+1,	1);
	// 返回值临时存放
	uint8_t content_1_return, content_2_return;
	// 把data1的内容先预处理成 16 位的数组，方便取用
	int16_t rx_data1_byte[4];
	memcpy(&rx_data1_byte, &rx_pack.data1, sizeof(rx_data1_byte)); 
	// 将内容存到临时结构体，尝试插入订阅列表
	subscribe_param_t param;
	memset(&param, 0, sizeof(param)); // 清零防止出 BUG
	param.content_id		= content_1_offset_id;
	param.subscribe_period	= rx_data1_byte[0];
	param.subscribe_times	= rx_data1_byte[1];
	param.pValueAdd			= steering_communication_Subscribe_GetValueAdd(steering, content_1_offset_id);
	content_1_return = steering_communication_SubscribeList_TryIncert(steering_communication_subscribe_list, param);
	
	memset(&param, 0, sizeof(param)); // 清零防止出 BUG
	param.content_id		= content_2_offset_id;
	param.subscribe_period	= rx_data1_byte[2];
	param.subscribe_times	= rx_data1_byte[3];
	param.pValueAdd			= steering_communication_Subscribe_GetValueAdd(steering, content_2_offset_id);
	content_2_return = steering_communication_SubscribeList_TryIncert(steering_communication_subscribe_list, param);
	// 添加返回标志位
	uint16_t sum_return = content_1_return | content_2_return<<8;
	rx_pack.data2	= sum_return;
	rx_pack.cmd_id	= RETURN_CMD_ID;
	return rx_pack;
}



/*
	用于尝试将某一待订阅内容剔除出订阅消息列表
*/
STEERING_COMMUNICATION_RETURN_T steering_communication_SubscribeList_TryDelete(steering_communication_subscribe_list_unit_t *list, SUBSCRIBE_CONTENT_X_OFFSET_ID_t id)
{
	for (int i=0; i<SUBSCRIBE_LIST_MAXIMUM_LENGTH; i++) // 遍历订阅列表
	{
		if (list[i].param.content_id == id) // 找到直接删掉
		{
			memset(&list[i], 0, sizeof(list[i]));
			return STEERING_COMMUNICATION_OK;
		}
	}
	return STEERING_COMMUNICATION_SUBCRIBE_LIST_NOT_SUBSCRIBE_YET;
}

/*
	用于处理 cmd_id 为 DELETE_SUBSCRIBE_VALUE 的情况
*/
steering_communication_pack_t steering_communication_DELETE_SUBSCRIBED_VALUE_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	// 先判断 subscribe_delete_offset_id 是否为 0
	rx_pack.cmd_id	= RETURN_CMD_ID;
	if (!rx_pack.data2)
	{
		uint8_t cnt = 8;
		while (cnt) // 逐 Bit 判断
		{
			if (rx_pack.data2 & (2^(8-cnt)))
				memset(&steering_communication_subscribe_list[8-cnt], 0, sizeof(steering_communication_subscribe_list[8-cnt])); // 直接删光光
			cnt--;
		}
		return rx_pack;	
	}
	else
	{
		for (int i=0; i<8; i++) // 遍历数据区1
		{
			if (rx_pack.data1[i]) // 里面内容不是0
				steering_communication_SubscribeList_TryDelete(steering_communication_subscribe_list, rx_pack.data1[i]); // 尝试删除
		}
		return rx_pack;	
	}


}

/*
		用于处理 cmd_id 为 SET_VELOCITY_VECTOR 的情况
*/

steering_communication_pack_t steering_communication_SET_VELOCITY_VECTOR_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	// 修改为SET_VELOCITY_VECTOR的CMDID
	rx_pack.cmd_id = SET_VELOCITY_VECTOR;
	int16_t rx_data1_byte[4];
	memcpy(rx_data1_byte, rx_pack.data1, sizeof(rx_data1_byte));
	memcpy(chassis_power_control.scaled_power_32, rx_data1_byte+2, 4);
	Steering_Wheel_SetProtocolPosition(steering, rx_data1_byte[0]);
	Steering_Wheel_SetProtocolSpeed(steering,rx_data1_byte[1]);
	if (chassis_power_control.scaled_power_32 != VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY)
	{
		chassis_power_control.motor_control_flag = 1;
	}
		return rx_pack;
}
/*
		用于处理cmd_id 为 DISABLE_CONTROLLING 的情况
*/
steering_communication_pack_t steering_communication_SET_DISABLE_OPERATION_MODE_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	rx_pack.cmd_id	= RETURN_CMD_ID;
	uint8_t deg_mode,arc_mode;
	memcpy(&deg_mode,(uint8_t)&rx_pack.data2,1);
	Steering_Wheel_SetProtocolDegMode(steering,deg_mode);
	memcpy(&arc_mode,(uint8_t)&rx_pack.data2+1,1);
	Steering_Wheel_SetProtocolArcMode(steering,arc_mode);
	steering->parameter.enable=DISABLE_CONTROLLING;
	return rx_pack;
}
/*
	用于处理cmd_id 为 ENABLE_CONTROLLING 的情况
*/
steering_communication_pack_t steering_communication_SET_ENABLE_OPERATION_MODE_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	rx_pack.cmd_id	= RETURN_CMD_ID;
	uint8_t deg_mode,arc_mode;
	memcpy(&deg_mode,(uint8_t)&rx_pack.data2,1);
	Steering_Wheel_SetProtocolDegMode(steering,deg_mode);
	memcpy(&arc_mode,(uint8_t)&rx_pack.data2+1,1);
	Steering_Wheel_SetProtocolArcMode(steering,arc_mode);
	steering->parameter.enable=ENABLE_CONTROLLING;
	return rx_pack;
}

/*
	用于处理cmd_id 为 CHECK_SUBSCRIBE_LIST 的情况
*/

steering_communication_pack_t steering_communication_CHECK_SUBSCRIBE_LIST_handler(steering_wheel_t *steering, steering_communication_pack_t rx_pack)
{
	rx_pack.cmd_id	= RETURN_CMD_ID;
	for(int i;i<SUBSCRIBE_LIST_MAXIMUM_LENGTH;i++)
	{
		memcpy(&rx_pack.data1[i],&steering_communication_subscribe_list[i].param,1);
	}
	return rx_pack;
}

/*
	供外部调用，用于初始化必要的外设与功能
*/
void steering_communication_init(void)
{
    /* Initialize transmission functions */
    steering_communication_ctx.tx_cmd =platform_trans;
	steering_communication_ctx.handle = &STEERING_COMMUNICATION_HANDLE;
    /* Initialize CAN driver interface */
    #if defined(STM32F105) | (STM32F407)
		steering_communication_CAN_TxHeaderStruct.StdId = 0;
        steering_communication_CAN_TxHeaderStruct.ExtId = 0;
        steering_communication_CAN_TxHeaderStruct.DLC = 8;
        steering_communication_CAN_TxHeaderStruct.IDE = CAN_ID_EXT; // 使用拓展帧
        steering_communication_CAN_TxHeaderStruct.RTR = CAN_RTR_DATA;
        steering_communication_CAN_TxHeaderStruct.TransmitGlobalTime = DISABLE;
    #endif
	Steering_Communication_SubscribeList_Init(); // 初始化订阅队列
}


/*
	供外部调用，处理 CAN 回传的函数
*/
STEERING_COMMUNICATION_RETURN_T steering_communication_rx_handler(uint32_t extid, uint8_t data1[])
{
	steering_communication_pack_t pack;
	steering_wheel_t *temp_handle;
	pack = steering_communication_receive_unpack(extid, data1);
	temp_handle=Steering_FindSteeringHandle_via_CANID(pack.steering_id);
	if (temp_handle==NULL)
	{
		return STEERING_COMMUNICATION_WRONG_PARAM;
	}
		else
		{
	switch (pack.cmd_id)
	{
		case GET_PID_PARAMETER:
			pack = steering_communication_GET_PID_PARAMETER_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case SET_PID_PARAMETER:
			pack = steering_communication_SET_PID_PARAMETER_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case ADD_SUBSCRIBE_VALUE:
			pack = steering_communication_ADD_SUBSCRIBE_VALUE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case DELETE_SUBSCRIBED_VALUE:
			pack = steering_communication_DELETE_SUBSCRIBED_VALUE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case SET_VELOCITY_VECTOR:
			pack = steering_communication_SET_VELOCITY_VECTOR_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case DISABLE_CONTROLLING:
			pack = steering_communication_SET_DISABLE_OPERATION_MODE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case ENABLE_CONTROLLING :
			pack = steering_communication_SET_ENABLE_OPERATION_MODE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		case CHECK_SUBSCRIBE_LIST :
			pack = steering_communication_CHECK_SUBSCRIBE_LIST_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
			break;
		default:
			return STEERING_COMMUNICATION_WRONG_PARAM;
			break;
	}
	steering_communication_transmit(&steering_communication_ctx, &pack);
	return STEERING_COMMUNICATION_OK;
		}
	
//	switch (pack.cmd_id)
//	{
//		case GET_PID_PARAMETER:
//			pack = steering_communication_GET_PID_PARAMETER_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case SET_PID_PARAMETER:
//			pack = steering_communication_SET_PID_PARAMETER_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case ADD_SUBSCRIBE_VALUE:
//			pack = steering_communication_ADD_SUBSCRIBE_VALUE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case DELETE_SUBSCRIBED_VALUE:
//			pack = steering_communication_DELETE_SUBSCRIBED_VALUE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case SET_VELOCITY_VECTOR:
//			pack = steering_communication_SET_VELOCITY_VECTOR_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case DISABLE_CONTROLLING:
//			pack = steering_communication_SET_DISABLE_OPERATION_MODE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case ENABLE_CONTROLLING :
//			pack = steering_communication_SET_ENABLE_OPERATION_MODE_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		case CHECK_SUBSCRIBE_LIST :
//			pack = steering_communication_CHECK_SUBSCRIBE_LIST_handler(Steering_FindSteeringHandle_via_CANID(pack.steering_id), pack);
//			break;
//		default:
//			return STEERING_COMMUNICATION_WRONG_PARAM;
//			break;
//	}

//	if (Steering_FindSteeringHandle_via_CANID(pack.steering_id)==NULL)
//	{
//		return STEERING_COMMUNICATION_ERROR;
//	}
//	else
//	{
//	steering_communication_transmit(&steering_communication_ctx, &pack);
//	return STEERING_COMMUNICATION_OK;
//	}
}
/*
	订阅消息调度处
*/
	uint8_t fill_cnt=0;

STEERING_COMMUNICATION_RETURN_T steering_communication_SubscribeList_Scheduler(steering_wheel_t *steering)
{
	steering_communication_pack_t pack; // 待发送的数据包
	// 填入基本的数据包参数
	pack.steering_id	= steering->parameter.CANID;
	pack.cmd_id			= SUBSCRIBE_RETURN_CMD_ID;
	for (int i=0; i<SUBSCRIBE_LIST_MAXIMUM_LENGTH; i++) // 查找已订阅消息
	{
		if (steering_communication_subscribe_list[i].param.content_id != 0) // 找到了某个消息被订阅了
		{
			if (steering_communication_subscribe_list[i].remained_ticks-1 == 0) // 时间调度，如果到了一个周期
			{
				if (steering_communication_subscribe_list[i].subscribe_times_infinite_flag || steering_communication_subscribe_list[i].remained_times) // 如果是无限发送；或不是无限发送，但还有次数
				{
					// 复制数据
					memcpy(&pack.data1 + fill_cnt*4,	steering_communication_subscribe_list[i].param.pValueAdd,	1);
					memcpy(&pack.data2 + fill_cnt,		&steering_communication_subscribe_list[i].param.content_id, 1);
					// 不是无限发送，就要控制次数
					if (!steering_communication_subscribe_list[i].subscribe_times_infinite_flag) 
						steering_communication_subscribe_list[i].remained_times--;
					fill_cnt++; // 标志已经填充
				}
				else // 订阅次数耗尽，清空订阅
				{
					memset(&steering_communication_subscribe_list[i], 0, sizeof(steering_communication_subscribe_list[i]));
				}
					
			}
			else // 时间调度，还没到一个周期，继续及时
			{
				steering_communication_subscribe_list[i].remained_ticks--;
			}
		}
		if (fill_cnt == 2) // 两个消息就会装满一个包，开始发送
		{
			steering_communication_transmit(&steering_communication_ctx, &pack);
			// 发送完要清空数据区
			memset(&pack.data1, 0, sizeof(pack.data1)); 
			memset(&pack.data2, 0, sizeof(pack.data2)); 
			fill_cnt = 0; // 发送完以后清空计数
		}
	}
	if (fill_cnt) // 遍历完 ，还有没有发送的包
	{
		// 把剩下的一个数据单独发送
		steering_communication_transmit(&steering_communication_ctx, &pack);
	}
	
}

void steering_communication_queue_init(steering_communication_queue_t *queue)
{
	memset(queue, 0, sizeof(steering_communication_queue_t));
	queue->boundary = STEERING_COMMUNICATION_QUEUE_LENGTH;
}
/*
	用于将某个待发送队列中的pack发送了，还没写好
*/
void steering_communication_tx_queue_pop(steering_communication_ctx_t *ctx)
{

}
/*
	用于将某个pack放入待发送队列中，还没写好
*/
void steering_communication_queue_push(steering_communication_queue_t *queue, steering_communication_pack_t pack)
{
	//if (queue->tail != queue->)
}

static uint8_t platform_trans(void *handle, uint32_t Ext_CANID, uint8_t aData[])
{
    #if defined(STM32F105) | (STM32F407)
        steering_communication_CAN_TxHeaderStruct.ExtId = Ext_CANID;
	while (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0)
	{}; // 等待邮箱清空
        return HAL_CAN_AddTxMessage(handle, &steering_communication_CAN_TxHeaderStruct, aData, &steering_communication_pTxMailbox);
    #endif
}
