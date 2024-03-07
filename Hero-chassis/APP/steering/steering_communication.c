#include "steering_communication.h"
#define STM32F105

#if defined(STM32F105) | (STM32F407)
#include "can.h"
#endif

/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
CAN_TxHeaderTypeDef steering_communication_CAN_TxHeaderStruct;
uint32_t steering_communication_pTxMailbox;
#endif

static uint8_t platform_trans(void *handle, uint32_t Ext_CANID, uint8_t aData[]);
static uint8_t platform_get_msg(void *handle, uint32_t Ext_CANID, uint8_t aData[]);
STEERING_COMMUNICATION_RETURN_T steering_communication_ADD_SUBSCRIBE_VALUE_handler(subscribed_data_t *subscribed_data,
                                                                                                                              steering_communication_pack_t rx_pack);
																					
static uint8_t platform_trans(void *handle, uint32_t Ext_CANID, uint8_t aData[])
{
#if defined(STM32F105) | (STM32F407)
    steering_communication_CAN_TxHeaderStruct.ExtId = Ext_CANID;
		while (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0);
    return HAL_CAN_AddTxMessage(handle, &steering_communication_CAN_TxHeaderStruct, aData, &steering_communication_pTxMailbox);
#endif
}			



steering_communication_ctx_t steering_communication_ctx;
subscribed_data_t subscribed_data;
steering_communication_pack_t steering_pack;



STEERING_COMMUNICATION_RETURN_T steering_communication_SET_VELOCITY_VECTOR(steering_communication_MscB_id_t MscB,
                                                                           int16_t ProtocolPosition, int16_t ProtocolSpeed,
                                                                           float scaled_power_coefficient_32,
                                                                           steering_communication_pack_t tx_pack)
{
    tx_pack.cmd_id = SET_VELOCITY_VECTOR;
    tx_pack.steering_id = MscB;
    memcpy(&tx_pack.data1, &ProtocolPosition, sizeof(ProtocolPosition));

    memcpy(&tx_pack.data1[2], &ProtocolSpeed, sizeof(ProtocolSpeed));

    memcpy(&tx_pack.data1[4], &scaled_power_coefficient_32, sizeof(scaled_power_coefficient_32));

    steering_communication_transmit(&steering_communication_ctx, &tx_pack);
    return STEERING_COMMUNICATION_OK;
}

/**
 * @brief 订阅数据
 *
 * @param MscB
 * @param subscribe_content_2_offset_id
 * @param subscribe_content_1_offset_id
 * @param callback_cnt2
 * @param callback_term2
 * @param callback_cnt1
 * @param callback_term1
 * @param tx_pack
 * @return STEERING_COMMUNICATION_RETURN_T
 */
STEERING_COMMUNICATION_RETURN_T steering_communication_ADD_SUBSCRIBE_VALUE(steering_communication_MscB_id_t MscB,
                                                                           uint8_t subscribe_content_2_offset_id, uint8_t subscribe_content_1_offset_id, uint16_t callback_cnt2, uint16_t callback_term2, uint16_t callback_cnt1, uint16_t callback_term1, steering_communication_pack_t tx_pack)
{
    tx_pack.cmd_id = ADD_SUBSCRIBE_VALUE;
    tx_pack.steering_id = MscB;

    tx_pack.data2 = subscribe_content_2_offset_id << 8 | subscribe_content_1_offset_id;

    tx_pack.steering_id = MscB;

    memcpy(&tx_pack.data1, &callback_term1, sizeof(callback_term1));

    memcpy(&tx_pack.data1[2], &callback_cnt1, sizeof(callback_cnt1));

    memcpy(&tx_pack.data1[4], &callback_term2, sizeof(callback_term2));

    memcpy(&tx_pack.data1[6], &callback_cnt2, sizeof(callback_cnt2));
    steering_communication_transmit(&steering_communication_ctx, &tx_pack);
    return STEERING_COMMUNICATION_OK;
}

STEERING_COMMUNICATION_RETURN_T steering_communication_SET_VELOCITY_VECTOR_handler(
    steering_communication_pack_t rx_pack)
{

    switch (rx_pack.steering_id)
    {
    case A_MscB:
        chassis_power_control.all_mscb_ready_flag |= 0x01;
        memcpy(&chassis_power_control.expect_power_32[0], rx_pack.data1, sizeof(rx_pack.data1));
        break;
    case B_MscB:
        chassis_power_control.all_mscb_ready_flag |= 0x02;
        memcpy(&chassis_power_control.expect_power_32[1], rx_pack.data1, sizeof(rx_pack.data1));
        break;
    case C_MscB:
        chassis_power_control.all_mscb_ready_flag |= 0x04;
        memcpy(&chassis_power_control.expect_power_32[2], rx_pack.data1, sizeof(rx_pack.data1));
        break;
    case D_MscB:
        chassis_power_control.all_mscb_ready_flag |= 0x08;
        memcpy(&chassis_power_control.expect_power_32[3], rx_pack.data1, sizeof(rx_pack.data1));
        break;
    default:
        break;
    }

    return STEERING_COMMUNICATION_OK;
}

/*
    供外部调用，用于初始化必要的外设与功能
*/
void steering_communication_init(void)
{
    /* Initialize transmission functions */
    steering_communication_ctx.tx_cmd = platform_trans;
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
}


/*
    供外部调用，处理 CAN 回传的函数
*/
STEERING_COMMUNICATION_RETURN_T steering_communication_rx_handler(uint32_t extid, uint8_t data1[])
{
    steering_communication_pack_t pack;
    // steering_wheel_t *temp_handle;
    pack = steering_communication_receive_unpack(extid, data1);
    // temp_handle = Steering_FindSteeringHandle_via_CANID(pack.steering_id);
    //  if (temp_handle == NULL)
    //  {
    //      return STEERING_COMMUNICATION_WRONG_PARAM;
    //  }
    //  else
    //  {
    switch (pack.cmd_id)
    {
    case SUBSCRIBE_RETURN_CMD_ID:
        steering_communication_ADD_SUBSCRIBE_VALUE_handler(&subscribed_data,pack);
        break;
   case SET_VELOCITY_VECTOR:
        steering_communication_SET_VELOCITY_VECTOR_handler(pack);
        break;


     }
            return STEERING_COMMUNICATION_OK;
}











/**
 * @brief 处理回传的已订阅消息
 *
 * @param MscB
 * @param subscribe_content_2_offset_id
 * @param subscribe_content_1_offset_id
 * @param callback_cnt2
 * @param callback_term2
 * @param callback_cnt1
 * @param callback_term1
 * @param tx_pack
 * @return STEERING_COMMUNICATION_RETURN_T
 */

STEERING_COMMUNICATION_RETURN_T steering_communication_ADD_SUBSCRIBE_VALUE_handler(subscribed_data_t *subscribed_data,
                                                                                                                              steering_communication_pack_t rx_pack)

{
    uint8_t subscribe_content_1_offset_id, subscribe_content_2_offset_id;
    uint8_t data1[8];
    subscribe_content_1_offset_id = rx_pack.data2;
    subscribe_content_2_offset_id = rx_pack.data2>>8;


    memcpy(data1, rx_pack.data1, sizeof(rx_pack.data1));
    switch (rx_pack.steering_id)
    {
    case A_MscB:
        switch (subscribe_content_1_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[0] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[0] = data1[1] << 8 | data1[0];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[0] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[0] = data1[1] << 8 | data1[0];
            break;
        default:
            break;
        }

        switch (subscribe_content_2_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[0] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[0] = data1[5] << 8 | data1[4];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[0] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[0] = data1[5] << 8 | data1[4];
            break;
        default:
            break;
        }
        break;

    case B_MscB:
        switch (subscribe_content_1_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[1] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[1] = data1[1] << 8 | data1[0];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[1] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[1] = data1[1] << 8 | data1[0];
            break;
        default:
            break;
        }

        switch (subscribe_content_2_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[1] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[1] = data1[5] << 8 | data1[4];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[1] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[1] = data1[5] << 8 | data1[4];
            break;
        default:
            break;
        }
        break;

    case C_MscB:
        switch (subscribe_content_1_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[2] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[2] = data1[1] << 8 | data1[0];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[2] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[2] = data1[1] << 8 | data1[0];
            break;
        default:
            break;
        }

        switch (subscribe_content_2_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[2] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[2] = data1[5] << 8 | data1[4];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[2] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[2] = data1[5] << 8 | data1[4];
            break;
        default:
            break;
        }
        break;

    case D_MscB:
        switch (subscribe_content_1_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[3] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[3] = data1[1] << 8 | data1[0];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[3] = data1[1] << 8 | data1[0];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[3] = data1[1] << 8 | data1[0];
            break;
        default:
            break;
        }

        switch (subscribe_content_2_offset_id)
        {
        case motion_motor_rpm:
            subscribed_data->motion_motor_rpm[3] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_angle_lsb:
            subscribed_data->directive_motor_angle_lsb[3] = data1[5] << 8 | data1[4];
            break;

        case motion_motor_torque_current_lsb:
            subscribed_data->motion_motor_torque_current_lsb[3] = data1[5] << 8 | data1[4];
            break;

        case directive_motor_torque_current_lsb:
            subscribed_data->directive_motor_torque_current_lsb[3] = data1[5] << 8 | data1[4];
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return STEERING_COMMUNICATION_OK;
     }

     
     