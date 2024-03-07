/* Includes ------------------------------------------------------------------*/
#include "M3508.h"

#if defined(STM32F105) | (STM32F407)
    #include "can.h"
#endif


/* Motor Bus Settings --------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
    // F105 and F407 support two CAN buses, so two structures are defined
    M3508_motor_bus_t M3508_bus_1;
    M3508_motor_bus_t M3508_bus_2;
#endif

/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
    
    
    CAN_TxHeaderTypeDef M3508_CAN_TxHeaderStruct;
    uint32_t  M3508_pTxMailbox;
#endif

// Function prototypes
static uint8_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[]);
static uint8_t platform_get_msg(void *handle, uint16_t *CAN_ID, uint8_t aData[]);

/**
 * @brief Initialize M3508 motor control
 *
 * This function is used to initialize M3508 motor control, including configuring
 * the transmission functions and CAN interface.
 */
M3508_ctx_t M3508_ctx;

void M3508_Init(M3508_motor_bus_t *M3508_bus, void *bus)
{
    /* Initialize transmission functions */
    M3508_ctx.tx_cmd = platform_trans;
    /* Initialize CAN driver interface */
    #if defined(STM32F105) | (STM32F407)
        M3508_bus->handle = bus;
        M3508_CAN_TxHeaderStruct.ExtId = 0;
        M3508_CAN_TxHeaderStruct.DLC = 8;
        M3508_CAN_TxHeaderStruct.IDE = CAN_ID_STD;
        M3508_CAN_TxHeaderStruct.RTR = CAN_RTR_DATA;
        M3508_CAN_TxHeaderStruct.TransmitGlobalTime = DISABLE;
    #endif
}

/**
 * @brief Set current command for a single M3508 motor
 *
 * This function is used to set the current command for a single M3508 motor
 * and send it to the motor via the CAN bus.
 *
 * @param  M3508_bus  Pointer to the M3508 motor bus structure
 * @param  ESC_ID     ESC ID to determine which motor to control (range from 1~8, NOT CAN ID!!!)
 * @param  current    Current command value in LSB
 */
void M3508_set_single_motor_current(M3508_motor_bus_t *M3508_bus, uint8_t ESC_ID, uint16_t current, M3508_SINGLE_COMMAND_HOLD_t hold)
{
    // Pass the bus where the motor is located
    M3508_ctx.handle = M3508_bus->handle;
    // Pass the current value to the motor command structure
    M3508_bus->motor[ESC_ID].command.torque_current_lsb = current;
    // Call the CAN transmission function to send the motor command
    if (hold == SEND_COMMAND_NOW)
		M3508_command_transmit(&M3508_ctx, M3508_bus, (ESC_ID / 5 + 1));
}

/**
 * @brief Set current command for four M3508 motors
 *
 * This function is used to set the current command for four M3508 motors
 * and send them to the motors via the CAN bus.
 *
 * @param  M3508_bus  Pointer to the M3508 motor bus structure
 * @param  half       Half of the motors to control (0 or 1)
 * @param  current    Array of current command values for four motors in LSB
 */
void M3508_set_four_motor_current(M3508_motor_bus_t *M3508_bus, uint8_t half, uint16_t current[])
{
    // Pass the bus where the motors are located
    M3508_ctx.handle = M3508_bus->handle;
    uint8_t offset = 0;
    if (half) offset = 4;
    for (int i = 0; i < 4; i++)
        M3508_bus->motor[i + 1 + offset].command.torque_current_lsb = current[i];
    M3508_command_transmit(&M3508_ctx, M3508_bus, half + 1);
}

/**
 * @brief Set current command for all eight M3508 motors
 *
 * This function is used to set the current command for all eight M3508 motors
 * and send them to the motors via the CAN bus.
 *
 * @param  M3508_bus  Pointer to the M3508 motor bus structure
 * @param  current    Array of current command values for eight motors in LSB
 */
void M3508_set_all_motor_current(M3508_motor_bus_t *M3508_bus, uint16_t current[])
{
    // Pass the bus where the motors are located
    M3508_ctx.handle = M3508_bus->handle;
    for (int i = 0; i < 8; i++)
        M3508_bus->motor[i + 1].command.torque_current_lsb = current[i];
    M3508_command_transmit(&M3508_ctx, M3508_bus, 0);
}

/**
 * @brief Handle feedback from M3508 motors
 *
 * This function is used to handle feedback from M3508 motors, updating the
 * motor status information in the motor bus structure.
 *
 * @param  M3508_bus  Pointer to the M3508 motor bus structure
 */
M3508_RETURN_T M3508_feedback_handler(M3508_motor_bus_t *M3508_bus, uint16_t CAN_ID, uint8_t data[])
{
    M3508_ctx.handle = M3508_bus->handle;
    if (M3508_feedback_process(M3508_bus, CAN_ID, data))
		return M3508_ERROR;
	return M3508_OK;
}

/**
 * @brief Transmit data via CAN bus
 *
 * This function is used to transmit data via the CAN bus.
 *
 * @param  handle   Pointer to the CAN handle
 * @param  CAN_ID   CAN ID for the message
 * @param  aData    Array of data to transmit
 *
 * @return Status of the transmission (0 if successful, non-zero otherwise)
 */
static uint8_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[])
{
    #if defined(STM32F105) | (STM32F407)
        M3508_CAN_TxHeaderStruct.StdId = CAN_ID;
	
        return HAL_CAN_AddTxMessage(handle, &M3508_CAN_TxHeaderStruct, aData, &M3508_pTxMailbox);
    #endif
}

/**
 * @brief Receive a message via CAN bus
 *
 * This function is used to receive a message via the CAN bus.
 *
 * @param  handle   Pointer to the CAN handle
 * @param  CAN_ID   Pointer to store the received CAN ID
 * @param  aData    Array to store the received data
 *
 * @return Status of the reception (0 if successful, non-zero otherwise)
 */
static uint8_t platform_get_msg(void *handle, uint16_t *CAN_ID, uint8_t aData[])
{
    #if defined(STM32F105) | (STM32F407)
        int status;
        CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
        status = HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &CAN_RxHeaderStruct, aData);
        *CAN_ID = (uint16_t)CAN_RxHeaderStruct.StdId;
        return status;
    #endif
}
