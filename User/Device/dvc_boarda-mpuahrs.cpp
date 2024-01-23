/**
 * @file dvc_boarda-mpuahrs.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief A板自带MPU姿态传感器
 * @version 0.1
 * @date 2023-09-29 0.1 新增A板MPU库
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_boarda-mpuahrs.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief MPU姿态传感器初始化, 内含延时函数2390ms
 * 
 * @param hspi 指定的SPI
 */
void Class_BoardA_MPU::Init()
{
    SPI_Manage_Object = &SPI5_Manage_Object;
    CS_GPIOx = BoardA_MPU6500_CS_GPIO_Port;
    CS_GPIO_Pin = BoardA_MPU6500_CS_Pin;

    PID_Mahony_X.Init(1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    PID_Mahony_Y.Init(1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    PID_Mahony_Z.Init(1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    Filter_Fourier_Accelerate_X.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Accelerate_Y.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Accelerate_Z.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Temperature.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Omega_X.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Omega_Y.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Omega_Z.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Magnet_X.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Magnet_Y.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);
    Filter_Fourier_Magnet_Z.Init(-32767.0f, 32767.0f, Filter_Fourier_Type_LOWPASS, 1.0f, 500.0f, 1000.0f, 50);

    //初始化四元数与角度
    Data_Quaternion.q_0 = 1.0f;
    Data_Quaternion.q_1 = 0.0f;
    Data_Quaternion.q_2 = 0.0f;
    Data_Quaternion.q_3 = 0.0f;
    Pre_Angle.Angle_Yaw = 0.0f;
    Pre_Angle.Angle_Pitch = 0.0f;
    Pre_Angle.Angle_Roll = 0.0f;

    //配置MPU寄存器
    HAL_Delay(1000);
    Register_Init();

    //启动前静止记录偏置
    MPU_Offset_Config();
}

/**
 * @brief 读取IST寄存器内容, 与从机4绑定, 内含延时函数40ms
 * 
 * @param Address IST的寄存器地址
 * @return uint8_t 读取的数据
 */
uint8_t Class_BoardA_MPU::IST_Register_Read(uint8_t Address)
{
    uint8_t return_value;
    void *tmp_buffer;

    //MCU->MPU, 读取IST地址报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_REGISTER;
    //IST寄存器地址
    SPI_Manage_Object->Tx_Buffer[1] = Address;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 使能I2C从机4状态报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_CONTROL;
    //使能从机4
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Slave_4_Control_Register *>(tmp_buffer)->I2C_Slave_4_Enable = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    //MPU->MCU, 读取IST数据报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_DATA_IN | MPU6500_READ_MASK;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 1, 1);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    return_value = SPI_Manage_Object->Rx_Buffer[1];
    
    //MCU->MPU, 失能I2C从机4状态报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_CONTROL;
    //失能I2C从机4状态(置位0所以不用配)
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    return(return_value);
}

/**
 * @brief 写入IST寄存器内容, 与从机1绑定, 内含延时函数40ms
 * 
 * @param Address IST的寄存器地址
 * @param Data 写入的数据
 */
void Class_BoardA_MPU::IST_Register_Write(uint8_t Address, uint8_t Data)
{
    void *tmp_buffer;

    //MCU->MPU, 失能I2C从机1状态报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_CONTROL;
    //失能I2C从机1状态(置位0所以不用配)
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    //MCU->MPU, 写入IST地址报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_REGISTER;
    //IST寄存器地址
    SPI_Manage_Object->Tx_Buffer[1] = Address;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    //MCU->MPU, 写入IST数据报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_DATA_OUT;
    //IST寄存器数据
    SPI_Manage_Object->Tx_Buffer[1] = Data;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    //MCU->MPU, 使能I2C从机1状态报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_CONTROL;
    //使能从机1, 发送长度1
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Slave_x0123_Control_Register *>(tmp_buffer)->I2C_Slave_Enable = 1;
    reinterpret_cast<Struct_MPU6500_I2C_Slave_x0123_Control_Register *>(tmp_buffer)->I2C_Slave_Length = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
}

/**
 * @brief MPU本身配置, 内含延时函数40ms
 * 
 */
void Class_BoardA_MPU::MPU_IMU_Config()
{
    void *tmp_buffer;

    //MCU->MPU, 重置MPU报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_POWER_MANAGEMENT_1;
    //失能MPU
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_Power_Management_1_Register *>(tmp_buffer)->Device_Reset = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 使能MPU报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_POWER_MANAGEMENT_1;
    //使能MPU(置位0所以不用配), 选择时钟为PLL获取
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_Power_Management_1_Register *>(tmp_buffer)->Clock_Select = 3;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    //MCU->MPU, MPU陀螺仪温度计加速度计的参数配置报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_CONFIG;
    //设置MPU温度计低通滤波器41Hz与42Hz
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_Config_Register *>(tmp_buffer)->Digital_Lowpass_Filter_Config = 4;
    //配置MPU量程±2000dps
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[2];
    reinterpret_cast<Struct_MPU6500_Gyro_Config_Register *>(tmp_buffer)->Gyro_Full_Scale_Select = 3;
    //配置加速度计量程±4g
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[3];
    reinterpret_cast<Struct_MPU6500_Accel_Config_1_Register *>(tmp_buffer)->Accel_Full_Scale_Select = 1;
    //设置加速度计低通滤波器92Hz
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[4];
    reinterpret_cast<Struct_MPU6500_Accel_Config_2_Register *>(tmp_buffer)->Accel_Digital_Lowpass_Filter_Config = 2;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 5, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, MPU的通信模式配置报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_USER_CONTROL;
    //使能I2C主机模式
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_User_Control_Register *>(tmp_buffer)->I2C_Master_Enable = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
}

/**
 * @brief MPU的I2C主机通信配置, 内含延时函数20ms
 * 
 */
void Class_BoardA_MPU::MPU_I2C_Master_Config()
{
    void *tmp_buffer;
    
    //MCU->MPU, MPU的I2C主机模式配置报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_USER_CONTROL;
    //使能I2C主机模式, 重置I2C从机模块
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_User_Control_Register *>(tmp_buffer)->I2C_Master_Enable = 1;
    reinterpret_cast<Struct_MPU6500_User_Control_Register *>(tmp_buffer)->I2C_Interface_Disable = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, MPU的I2C时钟配置报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_MASTER_CONTROL;
    //选择400kHz时钟
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Master_Control_Register *>(tmp_buffer)->I2C_Master_Clock = 13;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
}

/**
 * @brief MPU的I2C从机通信配置, 内含延时函数30ms
 * 
 */
void Class_BoardA_MPU::MPU_I2C_Slave_Config()
{
    //自动MPU->MCU, MPU的I2C从机0对MPU->IST绑定设备与自动传输寄存器报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_0_ADDRESS;
    //配置地址
    SPI_Manage_Object->Tx_Buffer[1] = IST8310_I2C_ADDRESS | IST8310_READ_MASK;
    //配置连续接收的起始地址
    SPI_Manage_Object->Tx_Buffer[2] = IST8310_MAGNET_X_0_7;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 3, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, MPU的I2C从机1对MPU->IST绑定设备报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_ADDRESS;
    //配置地址
    SPI_Manage_Object->Tx_Buffer[1] = IST8310_I2C_ADDRESS;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, MPU的I2C从机4对IST->MPU绑定设备报文构造
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_ADDRESS;
    //配置地址
    SPI_Manage_Object->Tx_Buffer[1] = IST8310_I2C_ADDRESS | IST8310_READ_MASK;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
}

/**
 * @brief 初始化IST寄存器配置信息, 顺序也很重要, 内含延时函数200ms
 * 从机4对IST->MPU, 从机1对MPU->IST
 * 
 */
void Class_BoardA_MPU::MPU_IST_Register_Config()
{
    uint8_t tmp_data;

    //重启IST
    reinterpret_cast<Struct_IST8310_Control_2_Register *>(&tmp_data)->Hardware_Restart = 1;
    IST_Register_Write(IST8310_CONTROL_2, tmp_data);
    tmp_data = 0;
    HAL_Delay(10);

    //暂时休眠模式(置位0所以不用配)
    IST_Register_Write(IST8310_CONTROL_1, tmp_data);
    tmp_data = 0;
    HAL_Delay(10);

    //低噪模式, xyz均16次均值
    reinterpret_cast<Struct_IST8310_Sample_Average_Register *>(&tmp_data)->Sample_Times_X_Z = 4;
    reinterpret_cast<Struct_IST8310_Sample_Average_Register *>(&tmp_data)->Sample_Times_Y = 4;
    IST_Register_Write(IST8310_SAMPLE_AVERAGE, tmp_data);
    tmp_data = 0;
    HAL_Delay(10);
    
    //常规脉宽, 没找到资料, 官方代码是这个数
    tmp_data = 0xc0;
    IST_Register_Write(IST8310_PULSE_DURATION_CONTROL, tmp_data);
    tmp_data = 0;
    HAL_Delay(10);
}

/**
 * @brief 配置自动传输模式, 内含延时函数100ms
 * 
 */
void Class_BoardA_MPU::MPU_IST_Auto_Communicate_Config()
{
    void *tmp_buffer;
    
    //MCU->MPU, 失能从机0传输
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_0_CONTROL;
    //配置参数(置位0所以不用配)
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 失能从机1传输
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_CONTROL;
    //配置参数(置位0所以不用配)
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 失能从机4传输
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_CONTROL;
    //配置参数(置位0所以不用配)
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 每8次MPU采样触发一次IST的I2C传输
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_4_CONTROL;
    //配置参数
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Slave_4_Control_Register *>(tmp_buffer)->I2C_Master_Delay_Control = 7;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 使能从机0和1的延时传输
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_MASTER_DELAY_CONTROL;
    //配置参数
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Master_Delay_Control_Register *>(tmp_buffer)->I2C_Slave_0_Delay_Enable = 1;
    reinterpret_cast<Struct_MPU6500_I2C_Master_Delay_Control_Register *>(tmp_buffer)->I2C_Slave_1_Delay_Enable = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 配置从机0的传输地址和寄存器内容
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_0_REGISTER;
    //配置参数
    SPI_Manage_Object->Tx_Buffer[1] = IST8310_MAGNET_X_0_7;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 配置从机1的传输寄存器内容
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_REGISTER;
    //配置参数
    SPI_Manage_Object->Tx_Buffer[1] = IST8310_CONTROL_1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 配置从机1的传输数据内容
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_DATA_OUT;
    //配置参数
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_IST8310_Control_1_Register *>(tmp_buffer)->Measure_Mode = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 使能从机0的自动传输, 用于触发采样
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_0_CONTROL;
    //配置参数
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Slave_x0123_Control_Register *>(tmp_buffer)-> I2C_Slave_Enable = 1;
    reinterpret_cast<Struct_MPU6500_I2C_Slave_x0123_Control_Register *>(tmp_buffer)-> I2C_Slave_Length = 6;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
    
    //MCU->MPU, 使能从机1的自动传输, 用于请求报文
    SPI_Manage_Object->Tx_Buffer[0] = MPU6500_I2C_SLAVE_1_CONTROL;
    //配置参数
    tmp_buffer = &SPI_Manage_Object->Tx_Buffer[1];
    reinterpret_cast<Struct_MPU6500_I2C_Slave_x0123_Control_Register *>(tmp_buffer)-> I2C_Slave_Enable = 1;
    reinterpret_cast<Struct_MPU6500_I2C_Slave_x0123_Control_Register *>(tmp_buffer)-> I2C_Slave_Length = 1;
    //发送报文并善后完成一轮
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 2, 0);
    HAL_Delay(10);
    memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);
}

/**
 * @brief 启动前静止记录角速度偏置, 内含延时函数500ms
 * 
 */
void Class_BoardA_MPU::MPU_Offset_Config()
{
    uint8_t *tmp_rx_buffer = SPI_Manage_Object->Rx_Buffer;
    int max_calculate_times = 500;
    
    // //加速度校正, 默认方向偏差不大进行, 迭代收敛
    // float tmp_accel_x_sum = 0;
    // float tmp_accel_y_sum = 0;
    // float tmp_accel_z_sum = 0;
    // for(int i = 0; i < max_calculate_times; i++)
    // {
    //     //MPU->MCU, 读取陀螺仪偏置报文构造
    //     SPI_Manage_Object->Tx_Buffer[0] = MPU6500_ACCEL_X_15_8 | MPU6500_READ_MASK;
    //     //发送报文并善后完成一轮
    //     SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 1, 6);
    //     HAL_Delay(0);
    //     memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

    //     //记录偏置量
    //     tmp_accel_x_sum += (int16_t)(tmp_rx_buffer[1] << 8 | tmp_rx_buffer[2]);
    //     tmp_accel_y_sum += (int16_t)(tmp_rx_buffer[3] << 8 | tmp_rx_buffer[4]);
    //     tmp_accel_z_sum += (int16_t)(tmp_rx_buffer[5] << 8 | tmp_rx_buffer[6]);
    // }
    

    //角速度校正
    float tmp_gyro_x_sum = 0;
    float tmp_gyro_y_sum = 0;
    float tmp_gyro_z_sum = 0;
    for(int i = 0; i < max_calculate_times; i++)
    {
        //MPU->MCU, 读取陀螺仪偏置报文构造
        SPI_Manage_Object->Tx_Buffer[0] = MPU6500_GYRO_X_15_8 | MPU6500_READ_MASK;
        //发送报文并善后完成一轮
        SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 1, 6);
        HAL_Delay(0);
        memset(SPI_Manage_Object->Tx_Buffer, 0, SPI_BUFFER_SIZE);

        //记录偏置量
        tmp_gyro_x_sum += (int16_t)(tmp_rx_buffer[1] << 8 | tmp_rx_buffer[2]);
        tmp_gyro_y_sum += (int16_t)(tmp_rx_buffer[3] << 8 | tmp_rx_buffer[4]);
        tmp_gyro_z_sum += (int16_t)(tmp_rx_buffer[5] << 8 | tmp_rx_buffer[6]);
    }
    Gyro_X_Offset = tmp_gyro_x_sum / max_calculate_times;
    Gyro_Y_Offset = tmp_gyro_y_sum / max_calculate_times;
    Gyro_Z_Offset = tmp_gyro_z_sum / max_calculate_times;
}

/**
 * @brief 初始化寄存器配置信息, 顺序也很重要, 失能后要启动MPU再配置其他, 内含延时函数1390ms
 * 
 */
void Class_BoardA_MPU::Register_Init()
{
    //第一步, MPU本身配置
    MPU_IMU_Config();

    //第二步, MPU的I2C主机通信配置
    MPU_I2C_Master_Config();
    
    //第三步, MPU的I2C从机通信配置
    MPU_I2C_Slave_Config();

    //第四步, 自此开始配置IST的寄存器
    MPU_IST_Register_Config();

    //第五步, 配置自动传输模式
    MPU_IST_Auto_Communicate_Config();
}

/**
 * @brief 位姿解算
 * 
 */
void Class_BoardA_MPU::Pose_Calculation()
{
    //加速度计的加速度
    float temp_accel_norm, temp_accel_x, temp_accel_y, temp_accel_z;
    //当前角度计算的加速度计理论分量值与加速度计实际三轴分量的偏差
    float accel_error_x, accel_error_y, accel_error_z;
    //四元数的重力加速度
    float pose_x, pose_y, pose_z;

    //暂存的陀螺仪的角速度
    float temp_omega_x, temp_omega_y, temp_omega_z;

    //暂存的四元数
    float temp_q_norm, temp_q_0, temp_q_1, temp_q_2, temp_q_3;
    //备用计算的四元数
    float q_0_q_0, q_0_q_1, q_0_q_2, q_0_q_3, q_1_q_1, q_1_q_2, q_1_q_3, q_2_q_2, q_2_q_3, q_3_q_3;
    //下一状态的四元数
    float temp_q_norm_next, temp_q_0_next, temp_q_1_next, temp_q_2_next, temp_q_3_next;
    //下一状态备用计算的四元数
    float q_0_q_0_next, q_0_q_1_next, q_0_q_2_next, q_0_q_3_next, q_1_q_1_next, q_1_q_2_next, q_1_q_3_next, q_2_q_2_next, q_2_q_3_next, q_3_q_3_next;

    //规避除零
    if(Data_Accelerate.Accelerate_X == 0.0f && Data_Accelerate.Accelerate_Y == 0.0f && Data_Accelerate.Accelerate_Z == 0.0f)
    {
        return;
    }
    else
    {
        //暂存角速度
        temp_omega_x = Data_Omega.Omega_X;
        temp_omega_y = Data_Omega.Omega_Y;
        temp_omega_z = Data_Omega.Omega_Z;
        //暂存四元数
        temp_q_0 = Data_Quaternion.q_0;
        temp_q_1 = Data_Quaternion.q_1;
        temp_q_2 = Data_Quaternion.q_2;
        temp_q_3 = Data_Quaternion.q_3;
        //备用四元数计算值
        q_0_q_0 = temp_q_0 * temp_q_0;
        q_0_q_1 = temp_q_0 * temp_q_1;
        q_0_q_2 = temp_q_0 * temp_q_2;
        q_0_q_3 = temp_q_0 * temp_q_3;
        q_1_q_1 = temp_q_1 * temp_q_1;
        q_1_q_2 = temp_q_1 * temp_q_2;
        q_1_q_3 = temp_q_1 * temp_q_3;
        q_2_q_2 = temp_q_2 * temp_q_2;
        q_2_q_3 = temp_q_2 * temp_q_3;
        q_3_q_3 = temp_q_3 * temp_q_3;

        //加速度归一化
        arm_sqrt_f32(Data_Accelerate.Accelerate_X * Data_Accelerate.Accelerate_X + Data_Accelerate.Accelerate_Y * Data_Accelerate.Accelerate_Y + Data_Accelerate.Accelerate_Z * Data_Accelerate.Accelerate_Z, &temp_accel_norm);
        temp_accel_x = Data_Accelerate.Accelerate_X / temp_accel_norm;
        temp_accel_y = Data_Accelerate.Accelerate_Y / temp_accel_norm;
        temp_accel_z = Data_Accelerate.Accelerate_Z / temp_accel_norm;
        //计算四元数的重力
        pose_x = 2.0f * (q_1_q_3 - q_0_q_2);
        pose_y = 2.0f * (q_0_q_1 + q_2_q_3);
        pose_z = 2.0f * (q_0_q_0 + q_3_q_3) - 1;
        // pose_z = q_0_q_0 - q_1_q_1 - q_2_q_2 + q_3_q_3;
        //计算加速度计单位化的重力与四元数的重力的外积确定差距大小
        accel_error_x = temp_accel_y * pose_z - temp_accel_z * pose_y;
        accel_error_y = temp_accel_z * pose_x - temp_accel_x * pose_z;
        accel_error_z = temp_accel_x * pose_y - temp_accel_y * pose_x;

        //Mahony算法增量式PI计算
        PID_Mahony_X.Set_Now(accel_error_x);
        PID_Mahony_X.Set_Target(0.0f);
        PID_Mahony_X.TIM_Adjust_PeriodElapsedCallback();
        temp_omega_x += PID_Mahony_X.Get_Out();
        PID_Mahony_Y.Set_Now(accel_error_y);
        PID_Mahony_Y.Set_Target(0.0f);
        PID_Mahony_Y.TIM_Adjust_PeriodElapsedCallback();
        temp_omega_y += PID_Mahony_Y.Get_Out();
        PID_Mahony_Z.Set_Now(accel_error_z);
        PID_Mahony_Z.Set_Target(0.0f);
        PID_Mahony_Z.TIM_Adjust_PeriodElapsedCallback();
        temp_omega_z += PID_Mahony_Z.Get_Out();

        //Mahony算法离散化到微分方程进行姿态更新
        //四元数微分方程
        temp_q_0_next = temp_q_0 + (-temp_q_1 * temp_omega_x - temp_q_2 * temp_omega_y - temp_q_3 * temp_omega_z) * MPU6500_CALCULATE_PERIOD_HALF;
        temp_q_1_next = temp_q_1 + (temp_q_0 * temp_omega_x + temp_q_2 * temp_omega_z - temp_q_3 * temp_omega_y) * MPU6500_CALCULATE_PERIOD_HALF;
        temp_q_2_next = temp_q_2 + (temp_q_0 * temp_omega_y - temp_q_1 * temp_omega_z - temp_q_3 * temp_omega_x) * MPU6500_CALCULATE_PERIOD_HALF;
        temp_q_3_next = temp_q_3 + (temp_q_0 * temp_omega_z + temp_q_1 * temp_omega_y - temp_q_2 * temp_omega_x) * MPU6500_CALCULATE_PERIOD_HALF;

        //善后工作
        //下一状态备用四元数计算值
        q_0_q_0_next = temp_q_0_next * temp_q_0_next;
        q_1_q_1_next = temp_q_1_next * temp_q_1_next;
        q_2_q_2_next = temp_q_2_next * temp_q_2_next;
        q_3_q_3_next = temp_q_3_next * temp_q_3_next;
        //对新的四元数单位化并存储
        arm_sqrt_f32(q_0_q_0_next + q_1_q_1_next + q_2_q_2_next + q_3_q_3_next, &temp_q_norm_next);
        temp_q_0_next /= temp_q_norm_next;
        temp_q_1_next /= temp_q_norm_next;
        temp_q_2_next /= temp_q_norm_next;
        temp_q_3_next /= temp_q_norm_next;
        //存储新的四元数
        Data_Quaternion.q_0 = temp_q_0_next;
        Data_Quaternion.q_1 = temp_q_1_next;
        Data_Quaternion.q_2 = temp_q_2_next;
        Data_Quaternion.q_3 = temp_q_3_next;
        
        //再次计算单位化后的下一状态备用四元数计算值
        q_0_q_0_next = temp_q_0_next * temp_q_0_next;
        q_0_q_1_next = temp_q_0_next * temp_q_1_next;
        q_0_q_2_next = temp_q_0_next * temp_q_2_next;
        q_0_q_3_next = temp_q_0_next * temp_q_3_next;
        q_1_q_1_next = temp_q_1_next * temp_q_1_next;
        q_1_q_2_next = temp_q_1_next * temp_q_2_next;
        q_1_q_3_next = temp_q_1_next * temp_q_3_next;
        q_2_q_2_next = temp_q_2_next * temp_q_2_next;
        q_2_q_3_next = temp_q_2_next * temp_q_3_next;
        q_3_q_3_next = temp_q_3_next * temp_q_3_next;

        //四元数到欧拉角
        //解算到角度
        Data_Angle.Angle_Yaw = atan2f(2.0f * (q_0_q_3_next + q_1_q_2_next), 1.0f - 2.0f * (q_2_q_2_next + q_3_q_3_next));
        Data_Angle.Angle_Pitch = atan2f(2.0f * (q_0_q_1_next + q_2_q_3_next), 1.0f - 2.0f * (q_1_q_1_next + q_2_q_2_next)) - PI;
        Data_Angle.Angle_Roll = asinf(2.0f * (q_0_q_2_next - q_1_q_3_next));
        //防止跳变, 计算扣圈
        float tmp_delta_angle;
        tmp_delta_angle = Data_Angle.Angle_Yaw - Pre_Angle.Angle_Yaw;
        if(tmp_delta_angle >= PI)
        {
            Data_Angle.Angle_Yaw -= 2.0f * PI;
        }
        else if(tmp_delta_angle <= -PI)
        {
            Data_Angle.Angle_Yaw += 2.0f * PI;
        }
        tmp_delta_angle = Data_Angle.Angle_Pitch - Pre_Angle.Angle_Pitch;
        if(tmp_delta_angle >= PI)
        {
            Data_Angle.Angle_Pitch -= 2.0f * PI;
        }
        else if(tmp_delta_angle <= -PI)
        {
            Data_Angle.Angle_Pitch += 2.0f * PI;
        }
        //善后工作
        Pre_Angle.Angle_Yaw = Data_Angle.Angle_Yaw;
        Pre_Angle.Angle_Pitch = Data_Angle.Angle_Pitch;
        Pre_Angle.Angle_Roll = Data_Angle.Angle_Roll;
    }
}

/**
 * @brief 数据处理过程
 * 
 */
void Class_BoardA_MPU::Data_Process()
{
    //数据处理过程
    uint8_t *tmp_tx_buffer = SPI_Manage_Object->Tx_Buffer;
    uint8_t *tmp_rx_buffer = SPI_Manage_Object->Rx_Buffer;

    if(tmp_tx_buffer[0] == (MPU6500_ACCEL_X_15_8 | MPU6500_READ_MASK) && SPI_Manage_Object->Now_Tx_Length == 1 && SPI_Manage_Object->Now_Rx_Length == 14)
    {
        //加速度温度角速度信息
        Raw_Data_Accelerate.Accelerate_X = -((int16_t)(tmp_rx_buffer[1] << 8 | tmp_rx_buffer[2]) - Accel_X_Offset);
        Raw_Data_Accelerate.Accelerate_Y = -((int16_t)(tmp_rx_buffer[3] << 8 | tmp_rx_buffer[4]) - Accel_Y_Offset);
        Raw_Data_Accelerate.Accelerate_Z = (int16_t)(tmp_rx_buffer[5] << 8 | tmp_rx_buffer[6]) - Accel_Z_Offset;

        Raw_Temperature = (int16_t)(tmp_rx_buffer[7] << 8 | tmp_rx_buffer[8]);

        Raw_Data_Omega.Omega_X = (int16_t)(tmp_rx_buffer[9] << 8 | tmp_rx_buffer[10]) - Gyro_X_Offset;
        Raw_Data_Omega.Omega_Y = (int16_t)(tmp_rx_buffer[11] << 8 | tmp_rx_buffer[12]) - Gyro_Y_Offset;
        Raw_Data_Omega.Omega_Z = (int16_t)(tmp_rx_buffer[13] << 8 | tmp_rx_buffer[14]) - Gyro_Z_Offset;
    }
    else if(tmp_tx_buffer[0] == (MPU6500_WHO_AM_I | MPU6500_READ_MASK) && SPI_Manage_Object->Now_Tx_Length == 1 && SPI_Manage_Object->Now_Rx_Length == 1)
    {
        //存活确认信息
        if(tmp_rx_buffer[SPI_Manage_Object->Now_Tx_Length] == MPU6500_ID)
        {
            MPU_Status = MPU_Status_ENABLE;
        }
        else
        {
            MPU_Status = MPU_Status_DISABLE;
        }
    }
    else if(tmp_tx_buffer[0] == (MPU6500_EXTEND_SENSOR_DATA_00 | MPU6500_READ_MASK) && SPI_Manage_Object->Now_Tx_Length == 1 && SPI_Manage_Object->Now_Rx_Length == 6)
    {
        //磁场强度信息
        Raw_Data_Magnet.Magnet_X = *(int16_t *)(&tmp_rx_buffer[1]);
        Raw_Data_Magnet.Magnet_Y = *(int16_t *)(&tmp_rx_buffer[3]);
        Raw_Data_Magnet.Magnet_Z = *(int16_t *)(&tmp_rx_buffer[5]);
    }
}

/**
 * @brief SPI通信交互回调函数
 * 
 * @param Tx_Data 发送的数据
 * @param Rx_Data 接收的数据
 */
void Class_BoardA_MPU::SPI_TxRxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer)
{
    Data_Process();
}

/**
 * @brief TIM定时器中断滤波器计算回调函数
 * 
 */
void Class_BoardA_MPU::TIM_Calculate_PeriodElapsedCallback()
{
    //滤波器计算, 并将重力加速度国际单位化

    Filter_Fourier_Accelerate_X.Set_Now(Raw_Data_Accelerate.Accelerate_X);
    Filter_Fourier_Accelerate_X.TIM_Adjust_PeriodElapsedCallback();
    Data_Accelerate.Accelerate_X = Filter_Fourier_Accelerate_X.Get_Out() / 32768.f * MPU6500_ACCEL_RANGE * GRAVITY_ACCELERATE;

    Filter_Fourier_Accelerate_Y.Set_Now(Raw_Data_Accelerate.Accelerate_Y);
    Filter_Fourier_Accelerate_Y.TIM_Adjust_PeriodElapsedCallback();
    Data_Accelerate.Accelerate_Y = Filter_Fourier_Accelerate_Y.Get_Out() / 32768.f * MPU6500_ACCEL_RANGE * GRAVITY_ACCELERATE;

    Filter_Fourier_Accelerate_Z.Set_Now(Raw_Data_Accelerate.Accelerate_Z);
    Filter_Fourier_Accelerate_Z.TIM_Adjust_PeriodElapsedCallback();
    Data_Accelerate.Accelerate_Z = Filter_Fourier_Accelerate_Z.Get_Out() / 32768.f * MPU6500_ACCEL_RANGE * GRAVITY_ACCELERATE;

    //滤波器计算, 并将温度国际单位化

    Filter_Fourier_Temperature.Set_Now(Raw_Temperature);
    Filter_Fourier_Temperature.TIM_Adjust_PeriodElapsedCallback();
    Temperature = 21.0f + Filter_Fourier_Temperature.Get_Out() / 333.87f;

    //滤波器计算, 并将角速度国际单位化
    float tmp_omega;
    Filter_Fourier_Omega_X.Set_Now(Raw_Data_Omega.Omega_X);
    Filter_Fourier_Omega_X.TIM_Adjust_PeriodElapsedCallback();
    tmp_omega = Filter_Fourier_Omega_X.Get_Out();
    if(Math_Abs(tmp_omega) < 3)
    {
        tmp_omega = 0.0f;
    }
    Data_Omega.Omega_X = tmp_omega / 32768.0f * MPU6500_GYRO_RANGE * DEG_TO_RAD;

    Filter_Fourier_Omega_Y.Set_Now(Raw_Data_Omega.Omega_Y);
    Filter_Fourier_Omega_Y.TIM_Adjust_PeriodElapsedCallback();
    tmp_omega = Filter_Fourier_Omega_Y.Get_Out();
    if(Math_Abs(tmp_omega) < 3)
    {
        tmp_omega = 0.0f;
    }
    Data_Omega.Omega_Y = Filter_Fourier_Omega_Y.Get_Out() / 32768.0f * MPU6500_GYRO_RANGE * DEG_TO_RAD;

    Filter_Fourier_Omega_Z.Set_Now(Raw_Data_Omega.Omega_Z);
    Filter_Fourier_Omega_Z.TIM_Adjust_PeriodElapsedCallback();
    tmp_omega = Filter_Fourier_Omega_Z.Get_Out();
    if(Math_Abs(tmp_omega) < 1.0f)
    {
        tmp_omega = 0.0f;
    }
    Data_Omega.Omega_Z = Filter_Fourier_Omega_Z.Get_Out() / 32768.0f * MPU6500_GYRO_RANGE * DEG_TO_RAD;

    //滤波器计算, 并将角速度单位向量化

    Filter_Fourier_Magnet_X.Set_Now(Raw_Data_Magnet.Magnet_X);
    Filter_Fourier_Magnet_X.TIM_Adjust_PeriodElapsedCallback();
    Data_Magnet.Magnet_X = Filter_Fourier_Magnet_X.Get_Out();

    Filter_Fourier_Magnet_Y.Set_Now(Raw_Data_Magnet.Magnet_Y);
    Filter_Fourier_Magnet_Y.TIM_Adjust_PeriodElapsedCallback();
    Data_Magnet.Magnet_Y = Filter_Fourier_Magnet_Y.Get_Out();

    Filter_Fourier_Magnet_Z.Set_Now(Raw_Data_Magnet.Magnet_Z);
    Filter_Fourier_Magnet_Z.TIM_Adjust_PeriodElapsedCallback();
    Data_Magnet.Magnet_Z = Filter_Fourier_Magnet_Z.Get_Out();

    if(Data_Magnet.Magnet_X == 0.0f && Data_Magnet.Magnet_Y == 0.0f && Data_Magnet.Magnet_Z == 0.0f)
    {
        //规避除零
    }
    else
    {
        //规避除零
        float tmp_magnet_magnitude;
        arm_sqrt_f32(Data_Magnet.Magnet_X * Data_Magnet.Magnet_X + Data_Magnet.Magnet_Y * Data_Magnet.Magnet_Y + Data_Magnet.Magnet_Z * Data_Magnet.Magnet_Z, &tmp_magnet_magnitude);
        Data_Magnet.Magnet_X /= tmp_magnet_magnitude;
        Data_Magnet.Magnet_Y /= tmp_magnet_magnitude;
        Data_Magnet.Magnet_Z /= tmp_magnet_magnitude;
    }

    Pose_Calculation();
}

/**
 * @brief TIM定时器中断通信回调函数, 每个时间片至多发送125B以内数据
 * 
 */
void Class_BoardA_MPU::TIM100us_Send_PeriodElapsedCallback()
{
    TIM100us_Send_PeriodElapsedCallback_Mod10++;
    if (TIM100us_Send_PeriodElapsedCallback_Mod10 == 10)
    {
        TIM100us_Send_PeriodElapsedCallback_Mod10 = 0;
    }
    else if (TIM100us_Send_PeriodElapsedCallback_Mod10 == 1)
    {
        //MPU->MCU, 读取加速度温度角速度
        SPI5_Manage_Object.Tx_Buffer[0] = MPU6500_ACCEL_X_15_8 | MPU6500_READ_MASK;
        SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 1, 14);
    }
    else if (TIM100us_Send_PeriodElapsedCallback_Mod10 == 2)
    {
        //MPU->MCU, 读取自身在线情况
        SPI5_Manage_Object.Tx_Buffer[0] = MPU6500_WHO_AM_I | MPU6500_READ_MASK;
        SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 1, 1);
    }
    else if (TIM100us_Send_PeriodElapsedCallback_Mod10 == 3)
    {
        //MPU->MCU, 读取磁力计情况
        SPI5_Manage_Object.Tx_Buffer[0] = MPU6500_EXTEND_SENSOR_DATA_00 | MPU6500_READ_MASK;
        SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIOx, CS_GPIO_Pin, 1, 6);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
