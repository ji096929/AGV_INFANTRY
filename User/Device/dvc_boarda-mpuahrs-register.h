/**
 * @file dvc_bsp-mpuahrs-register.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief A板自带MPU姿态传感器
 * @version 0.1
 * @date 2023-09-29 0.1 新增A板MPU库
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_BSP_MPUAHRS_REGISTER_H
#define DVC_BSP_MPUAHRS_REGISTER_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

//此六个寄存器中的值表示在制造测试过程中产生的自测试输出, 此值用于检查最终用户执行的后续自测试输出
#define MPU6500_SELF_TEST_GYRO_X (0x00)
#define MPU6500_SELF_TEST_GYRO_Y (0x01)
#define MPU6500_SELF_TEST_GYRO_Z (0x02)
#define MPU6500_SELF_TEST_ACCEL_X (0x0d)
#define MPU6500_SELF_TEST_ACCEL_Y (0x0e)
#define MPU6500_SELF_TEST_ACCEL_Z (0x0f)

//此六个寄存器用于消除陀螺仪输出中的直流偏置, 在进入传感器寄存器之前，将此寄存器中的值添加到陀螺仪传感器值中
#define MPU6500_GYRO_X_OFFSET_15_8 (0x13)
#define MPU6500_GYRO_X_OFFSET_7_0 (0x14)
#define MPU6500_GYRO_Y_OFFSET_15_8 (0x15)
#define MPU6500_GYRO_Y_OFFSET_7_0 (0x16)
#define MPU6500_GYRO_Z_OFFSET_15_8 (0x17)
#define MPU6500_GYRO_Z_OFFSET_7_0 (0x18)

//该寄存器存储值除以内部采样率生成控制传感器数据输出速率的采样率, FIFO采样率
//此寄存器只有在FCHOICE = 0x11, 即FCHOICE_B寄存器位为0x00且0 < DLPF_CFG < 7时才有效
//采样率 = 内部采样率 / (1 + SAMPLER_DIVISION), 内部采样率 = 1kHz
#define MPU6500_SAMPLER_DIVISION (0x19)

//四个配置相关寄存器
#define MPU6500_CONFIG (0x1a)
#define MPU6500_GYRO_CONFIG (0x1b)
#define MPU6500_ACCEL_CONFIG_1 (0x1c)
#define MPU6500_ACCEL_CONFIG_2 (0x1d)

//低功率加速度计ORDER控制寄存器
#define MPU6500_LOW_POWER_ACCELLERATE_ORDER (0x1e)

//此寄存器保存x/y/z产生中断的加速度阈值
#define MPU6500_WAKEON_MOTION_THRESHOLD (0x1f)

//FIFO使能寄存器, 若置1则将对应数据以采样频率写入FIFO
#define MPU6500_FIFO_ENABLE (0x23)

//IIC主设备控制器
#define MPU6500_I2C_MASTER_CONTROL (0x24)

//IIC从设备相关寄存器
#define MPU6500_I2C_SLAVE_0_ADDRESS (0x25)
#define MPU6500_I2C_SLAVE_0_REGISTER (0x26)
#define MPU6500_I2C_SLAVE_0_CONTROL (0x27)
#define MPU6500_I2C_SLAVE_1_ADDRESS (0x28)
#define MPU6500_I2C_SLAVE_1_REGISTER (0x29)
#define MPU6500_I2C_SLAVE_1_CONTROL (0x2a)
#define MPU6500_I2C_SLAVE_2_ADDRESS (0x2b)
#define MPU6500_I2C_SLAVE_2_REGISTER (0x2c)
#define MPU6500_I2C_SLAVE_2_CONTROL (0x2d)
#define MPU6500_I2C_SLAVE_3_ADDRESS (0x2e)
#define MPU6500_I2C_SLAVE_3_REGISTER (0x2f)
#define MPU6500_I2C_SLAVE_3_CONTROL (0x30)
#define MPU6500_I2C_SLAVE_4_ADDRESS (0x31)
#define MPU6500_I2C_SLAVE_4_REGISTER (0x32)
#define MPU6500_I2C_SLAVE_4_DATA_OUT (0x33)
#define MPU6500_I2C_SLAVE_4_CONTROL (0x34)
#define MPU6500_I2C_SLAVE_4_DATA_IN (0x35)

//I2C主设备状态寄存器
#define MPU6500_I2C_MASTER_STATUS (0x36)

//三个中断相关寄存器
#define MPU6500_INTERRUPT_PIN_CONFIG (0x37)
#define MPU6500_INTERRUPT_ENABLE (0x38)
#define MPU6500_INTERRUPT_STATUS (0x3a)

//加速度温度角速度的原始数据
#define MPU6500_ACCEL_X_15_8 (0x3b)
#define MPU6500_ACCEL_X_7_0 (0x3c)
#define MPU6500_ACCEL_Y_15_8 (0x3d)
#define MPU6500_ACCEL_Y_7_0 (0x3e)
#define MPU6500_ACCEL_Z_15_8 (0x3f)
#define MPU6500_ACCEL_Z_7_0 (0x40)
#define MPU6500_TEMPERATURE_15_8 (0x41)
#define MPU6500_TEMPERATURE_7_0 (0x42)
#define MPU6500_GYRO_X_15_8 (0x43)
#define MPU6500_GYRO_X_7_0 (0x44)
#define MPU6500_GYRO_Y_15_8 (0x45)
#define MPU6500_GYRO_Y_7_0 (0x46)
#define MPU6500_GYRO_Z_15_8 (0x47)
#define MPU6500_GYRO_Z_7_0 (0x48)

//I2C从设备0~3通过辅助IIC接口从外部传感器读取的数据, 从机设备4读取的数据存放在I2C_SLAVE_4_DATA_IN中
#define MPU6500_EXTEND_SENSOR_DATA_00 (0x49)
#define MPU6500_EXTEND_SENSOR_DATA_01 (0x4a)
#define MPU6500_EXTEND_SENSOR_DATA_02 (0x4b)
#define MPU6500_EXTEND_SENSOR_DATA_03 (0x4c)
#define MPU6500_EXTEND_SENSOR_DATA_04 (0x4d)
#define MPU6500_EXTEND_SENSOR_DATA_05 (0x4e)
#define MPU6500_EXTEND_SENSOR_DATA_06 (0x4f)
#define MPU6500_EXTEND_SENSOR_DATA_07 (0x50)
#define MPU6500_EXTEND_SENSOR_DATA_08 (0x51)
#define MPU6500_EXTEND_SENSOR_DATA_09 (0x52)
#define MPU6500_EXTEND_SENSOR_DATA_10 (0x53)
#define MPU6500_EXTEND_SENSOR_DATA_11 (0x54)
#define MPU6500_EXTEND_SENSOR_DATA_12 (0x55)
#define MPU6500_EXTEND_SENSOR_DATA_13 (0x56)
#define MPU6500_EXTEND_SENSOR_DATA_14 (0x57)
#define MPU6500_EXTEND_SENSOR_DATA_15 (0x58)
#define MPU6500_EXTEND_SENSOR_DATA_16 (0x59)
#define MPU6500_EXTEND_SENSOR_DATA_17 (0x5a)
#define MPU6500_EXTEND_SENSOR_DATA_18 (0x5b)
#define MPU6500_EXTEND_SENSOR_DATA_19 (0x5c)
#define MPU6500_EXTEND_SENSOR_DATA_20 (0x5d)
#define MPU6500_EXTEND_SENSOR_DATA_21 (0x5e)
#define MPU6500_EXTEND_SENSOR_DATA_22 (0x5f)
#define MPU6500_EXTEND_SENSOR_DATA_23 (0x60)

//IIC从设备数据输出寄存器
#define MPU6500_I2C_SLAVE_0_DATA_OUT (0x63)
#define MPU6500_I2C_SLAVE_1_DATA_OUT (0x64)
#define MPU6500_I2C_SLAVE_2_DATA_OUT (0x65)
#define MPU6500_I2C_SLAVE_3_DATA_OUT (0x66)

#define MPU6500_I2C_MASTER_DELAY_CONTROL (0x67)
#define MPU6500_SIGNAL_PATH_RESET (0x68)
#define MPU6500_ACCEL_DETECT_CONTROL (0x69)
#define MPU6500_USER_CONTROL (0x6a)

//电源管理寄存器，用于配置MPU6500时钟源，控制传感器失能等
#define MPU6500_POWER_MANAGEMENT_1 (0x6b)
#define MPU6500_POWER_MANAGEMENT_2 (0x6c)

//记录写入到FIFO的字节数
#define MPU6500_FIFO_COUNT_15_8 (0x72)
#define MPU6500_FIFO_COUNT_7_0 (0x73)

//用于从FIFO缓冲区读写数据
#define MPU6500_FIFO_READ_WRITE (0x74)

//存储一个8位数据用于验证设备的标示, 一般是0x70
#define MPU6500_WHO_AM_I (0x75)

//此六个寄存器用于消除加速度计输出中的直流偏置, 在进入传感器寄存器之前, 将此寄存器中的值添加到加速度计传感器值中
#define MPU6500_X_ACCEL_OFFSET_14_7 (0x77)
#define MPU6500_X_ACCEL_OFFSET_6_0 (0x78)
#define MPU6500_Y_ACCEL_OFFSET_14_7 (0x7a)
#define MPU6500_Y_ACCEL_OFFSET_6_0 (0x7b)
#define MPU6500_Z_ACCEL_OFFSET_14_7 (0x7d)
#define MPU6500_Z_ACCEL_OFFSET_6_0 (0x7e)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MPU6500_CONFIG寄存器具体位段
 *
 */
struct Struct_MPU6500_Config_Register
{
    uint8_t Digital_Lowpass_Filter_Config : 3;
    uint8_t Extern_Synchronize_Set : 3;
    uint8_t FIFO_Mode : 1;
    uint8_t Reserved : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_GYRO_CONFIG寄存器具体位段
 *
 */
struct Struct_MPU6500_Gyro_Config_Register
{
    uint8_t Filter_Choice : 2;
    uint8_t Reserved : 1;
    uint8_t Gyro_Full_Scale_Select : 2;
    uint8_t Z_Gyro_Self_Test : 1;
    uint8_t Y_Gyro_Self_Test : 1;
    uint8_t X_Gyro_Self_Test : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_ACCEL_CONFIG_1寄存器具体位段
 *
 */
struct Struct_MPU6500_Accel_Config_1_Register
{
    uint8_t Reserved : 3;
    uint8_t Accel_Full_Scale_Select : 2;
    uint8_t Z_Gyro_Self_Test : 1;
    uint8_t Y_Gyro_Self_Test : 1;
    uint8_t X_Gyro_Self_Test : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_ACCEL_CONFIG_2寄存器具体位段
 *
 */
struct Struct_MPU6500_Accel_Config_2_Register
{
    uint8_t Accel_Digital_Lowpass_Filter_Config : 3;
    uint8_t Accel_Filter_Choice : 1;
    uint8_t Reserved : 4;
} __attribute__((packed));

/**
 * @brief MPU6500_LOW_POWER_ACCEL_ORDER寄存器具体位段
 *
 */
struct Struct_MPU6500_Low_Power_Accel_Order_Register
{
    uint8_t Low_Power_Oscillator_Clock_Select : 4;
    uint8_t Reserved : 4;
} __attribute__((packed));

/**
 * @brief MPU6500_FIFO_ENABLE寄存器具体位段
 *
 */
struct Struct_MPU6500_FIFO_Enable_Register
{
    uint8_t Slave_0 : 1;
    uint8_t Slave_1 : 1;
    uint8_t Slave_2 : 1;
    uint8_t Accel : 1;
    uint8_t Gyro_Z : 1;
    uint8_t Gyro_Y : 1;
    uint8_t Gyro_X : 1;
    uint8_t Temperature : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_I2C_MASTER_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_I2C_Master_Control_Register
{
    uint8_t I2C_Master_Clock : 4;
    uint8_t I2C_Master_Pause_Nonestop_Register : 1;
    uint8_t Slave_3_FIFO_Enable : 1;
    uint8_t Wait_For_External_Sensor : 1;
    uint8_t Multi_Master_Enable : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_I2C_SLAVE_0123_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_I2C_Slave_x0123_Control_Register
{
    uint8_t I2C_Slave_Length : 4;
    uint8_t I2C_Slave_Group : 1;
    uint8_t I2C_Slave_Register_Disband : 1;
    uint8_t I2C_Slave_Byte_Switch : 1;
    uint8_t I2C_Slave_Enable : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_I2C_SLAVE_4_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_I2C_Slave_4_Control_Register
{
    uint8_t I2C_Master_Delay_Control : 5;
    uint8_t I2C_Slave_Register_Disband : 1;
    uint8_t I2C_Slave_4_Done_Interrupt_Enable : 1;
    uint8_t I2C_Slave_4_Enable : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_I2C_MASTER_STATUS寄存器具体位段
 *
 */
struct Struct_MPU6500_I2C_Master_Status_Register
{
    uint8_t I2C_Slave_0_NACK : 1;
    uint8_t I2C_Slave_1_NACK : 1;
    uint8_t I2C_Slave_2_NACK : 1;
    uint8_t I2C_Slave_3_NACK : 1;
    uint8_t I2C_Slave_4_NACK : 1;
    uint8_t I2C_Lost_Arbitration : 1;
    uint8_t I2C_Slave_4_Done : 1;
    uint8_t Pass_Through : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_INTERRUPT_PIN_CONFIG寄存器具体位段
 *
 */
struct Struct_MPU6500_Interrupt_Pin_Config_Register
{
    uint8_t Reserved : 1;
    uint8_t Bypass_Enable : 1;
    uint8_t FSYNC_Pin_Interrupt_Mode_Enable : 1;
    uint8_t FSYNC_Pin_Active_Pole : 1;
    uint8_t Interrupt_Any_Read_Clear_Status : 1;
    uint8_t Interrupt_Pin_Level_Hold : 1;
    uint8_t Output_Mode : 1;
    uint8_t Interrupt_Pin_Activate_Pole : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_INTERRUPT_ENABLE寄存器具体位段
 *
 */
struct Struct_MPU6500_Interrupt_Enable_Register
{
    uint8_t Raw_Ready_Enable : 1;
    uint8_t Reserved_1 : 2;
    uint8_t FSYNC_Pin_Interrupt_Enable : 1;
    uint8_t FIFO_Overflow_Enable : 1;
    uint8_t Reserved_2 : 1;
    uint8_t Wakeup_Mode_Enable : 1;
    uint8_t Reserved_3 : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_INTERRUPT_STATUS寄存器具体位段
 *
 */
struct Struct_MPU6500_Interrupt_Status_Register
{
    uint8_t Raw_Ready_Enable : 1;
    uint8_t Reserved_1 : 2;
    uint8_t FSYNC_Pin_Interrupt_Enable : 1;
    uint8_t FIFO_Overflow_Enable : 1;
    uint8_t Reserved_2 : 1;
    uint8_t Wakeup_Mode_Enable : 1;
    uint8_t Reserved_3 : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_I2C_MASTER_DELAY_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_I2C_Master_Delay_Control_Register
{
    uint8_t I2C_Slave_0_Delay_Enable : 1;
    uint8_t I2C_Slave_1_Delay_Enable : 1;
    uint8_t I2C_Slave_2_Delay_Enable : 1;
    uint8_t I2C_Slave_3_Delay_Enable : 1;
    uint8_t I2C_Slave_4_Delay_Enable : 1;
    uint8_t Reserved : 2;
    uint8_t Delay_External_Sensor_Shadow : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_I2C_MASTER_DELAY_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_Signal_Path_Reset_Register
{
    uint8_t Temperature_Reset : 1;
    uint8_t Accel_Reset : 1;
    uint8_t Gyro_Reset : 1;
    uint8_t Reserved : 5;
} __attribute__((packed));

/**
 * @brief MPU6500_ACCEL_DETECT_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_Accel_Detect_Control_Register
{
    uint8_t Reserved : 6;
    uint8_t Accel_Wakeon_Compare_Sample_Mode : 1;
    uint8_t Accel_Wakeon_Motion_Enable : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_USER_CONTROL寄存器具体位段
 *
 */
struct Struct_MPU6500_User_Control_Register
{
    uint8_t Signal_Path_Reset : 1;
    uint8_t I2C_Master_Reset : 1;
    uint8_t FIFO_Reset : 1;
    uint8_t DMP_Reset : 1;
    uint8_t I2C_Interface_Disable : 1;
    uint8_t I2C_Master_Enable : 1;
    uint8_t FIFO_Enable : 1;
    uint8_t DMP_Enable : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_POWER_MANAGEMENT_1寄存器具体位段
 *
 */
struct Struct_MPU6500_Power_Management_1_Register
{
    uint8_t Clock_Select : 3;
    uint8_t Temperature_Disable : 1;
    uint8_t Gyro_Standby : 1;
    uint8_t Cycle_Sleep : 1;
    uint8_t Sleep_Enable : 1;
    uint8_t Device_Reset : 1;
} __attribute__((packed));

/**
 * @brief MPU6500_POWER_MANAGEMENT_2寄存器具体位段
 *
 */
struct Struct_MPU6500_Power_Management_2_Register
{
    uint8_t Disable_Z_Gyro : 1;
    uint8_t Disable_Y_Gyro : 1;
    uint8_t Disable_X_Gyro : 1;
    uint8_t Disable_Z_Accel : 1;
    uint8_t Disable_Y_Accel : 1;
    uint8_t Disable_X_Accel : 1;
    uint8_t Low_Power_Wakeup_Control : 2;
} __attribute__((packed));

/**
 * @brief MPU6500_x_ACCEL_OFFSET_6_0寄存器具体位段
 *
 */
struct Struct_MPU6500_x_ACCEL_OFFSET_6_0_Register
{
    uint8_t Reserved : 1;
    uint8_t Accel_Offset_6_0 : 7;
} __attribute__((packed));
 
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
