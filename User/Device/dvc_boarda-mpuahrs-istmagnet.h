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

#ifndef DVC_BOARDA_MPUAHRS_ISTMAGNET_H
#define DVC_BOARDA_MPUAHRS_ISTMAGNET_H

/* Includes ------------------------------------------------------------------*/

#include "drv_bsp-boarda.h"
#include "dvc_boarda-mpuahrs-istmagnet-register.h"

/* Exported macros -----------------------------------------------------------*/

//A板自带MPU姿态传感器的IST8310扩展IIC从设备中, IIC读数据需要的置位, 写数据不需要
#define IST8310_READ_MASK (0x80)

//A板自带MPU姿态传感器的IST8310中, 自身固定值, 位于WHO_AM_I寄存器
#define IST8310_ID (0x10)
//A板自带MPU姿态传感器的IST8310中, 自身地址
#define IST8310_I2C_ADDRESS (0x0e)

/* Exported types ------------------------------------------------------------*/
 
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
