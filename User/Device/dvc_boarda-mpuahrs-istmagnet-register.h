/**
 * @file dvc_boarda-mpuahrs-istmagnet.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief A板自带MPU姿态传感器的IST磁力计
 * @version 0.1
 * @date 2023-09-26 0.1 新增A板IST操作
 * 
 * @copyright USTC-RoboWalker (c) 2023
 * 
 */

#ifndef DVC_BOARDA_MPUAHRS_ISTMAGNET_REGISTER_H
#define DVC_BOARDA_MPUAHRS_ISTMAGNET_REGISTER_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

//存储一个8位数据用于验证设备的标示, 一般是0x10
#define IST8310_WHO_AM_I (0x00)
//自身状态值
#define IST8310_STATUS_1 (0x02)

//磁场强度的原始数据
#define IST8310_MAGNET_X_0_7 (0x03)
#define IST8310_MAGNET_X_15_8 (0x04)
#define IST8310_MAGNET_Y_0_7 (0x05)
#define IST8310_MAGNET_Y_15_8 (0x06)
#define IST8310_MAGNET_Z_0_7 (0x07)
#define IST8310_MAGNET_Z_15_8 (0x08)

//自身状态值
#define IST8310_STATUS_2 (0x09)

//控制配置值寄存器
#define IST8310_CONTROL_1 (0x0a)
#define IST8310_CONTROL_2 (0x0b)

//自检寄存器
#define IST8310_SELF_TEST (0x0c)

//温度寄存器
#define IST8310_TEMPERATURE_7_0 (0x1c)
#define IST8310_TEMPERATURE_15_8 (0x1d)

//采样平均寄存器
#define IST8310_SAMPLE_AVERAGE (0x41)
//脉宽寄存器
#define IST8310_PULSE_DURATION_CONTROL (0x42)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IST8310_STATUS_1寄存器具体位段
 *
 */
struct Struct_IST8310_Status_1_Register
{
    uint8_t Data_Ready : 1;
    uint8_t Data_Overflow : 1;
    uint8_t Reserved : 6;
} __attribute__((packed));

/**
 * @brief IST8310_STATUS_2寄存器具体位段
 *
 */
struct Struct_IST8310_Status_2_Register
{
    uint8_t Reserved_1 : 3;
    uint8_t Interrupt_Flag : 1;
    uint8_t Reserved_2 : 4;
} __attribute__((packed));

/**
 * @brief IST8310_CONTROL_1寄存器具体位段
 *
 */
struct Struct_IST8310_Control_1_Register
{
    uint8_t Measure_Mode : 4;
    uint8_t Reserved : 4;
} __attribute__((packed));

/**
 * @brief IST8310_CONTROL_2寄存器具体位段
 *
 */
struct Struct_IST8310_Control_2_Register
{
    uint8_t Hardware_Restart : 1;
    uint8_t Reserved_1 : 1;
    uint8_t Interrupt_Pin_Pole : 1;
    uint8_t Interrupt_Enable : 1;
    uint8_t Reserved_2 : 4;
} __attribute__((packed));

/**
 * @brief IST8310_SELF_TEST寄存器具体位段
 *
 */
struct Struct_IST8310_Self_Test_Register
{
    uint8_t Reserved_1 : 6;
    uint8_t Self_Test_Mode : 1;
    uint8_t Reserved_2 : 1;
} __attribute__((packed));

/**
 * @brief IST8310_SAMPLE_AVERAGE寄存器具体位段
 *
 */
struct Struct_IST8310_Sample_Average_Register
{
    uint8_t Sample_Times_X_Z : 3;
    uint8_t Sample_Times_Y : 3;
    uint8_t Reserved : 2;
} __attribute__((packed));
 
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

