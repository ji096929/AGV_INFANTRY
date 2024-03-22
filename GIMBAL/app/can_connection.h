/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-11 00:37:51
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-11 03:29:24
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\app\can_connection.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef CAN_CONNECTION_H_
#define CAN_CONNECTION_H_

#include "drv_can.h"




typedef __packed enum
{
    CHASSIS_MODE_NOFORCE    =   0x00u,
    CHASSIS_MODE_TOPANGLE    =   0x02u,
    CHASSIS_MODE_ABSOLUTE   = 0x01u,
    CHASSIS_MODE_PRECISE    = 0x03u,
}CHASSIS_MODE_E;

typedef __packed struct
{
    float vx;
    float vy;
    float vw;
}CHASSIS_VELOCITY_T;


typedef __packed struct 
{
    CHASSIS_MODE_E mode;
    CHASSIS_VELOCITY_T velocity;
    bool follow_flag;
    bool invert_flag;

}CHASSIS_SEND_T;

typedef  __packed struct 
{
    CHASSIS_MODE_E mode;
    CHASSIS_VELOCITY_T velocity;


}CHASSIS_RECEIVE_T;

typedef __packed struct
{
    CHASSIS_SEND_T send;
    CHASSIS_RECEIVE_T receive;

}CHASSIS_T;
void Chassis_Connection_Init(void);
extern CHASSIS_T chassis;
void Chassis_Connection_Task(void);
void Can_Send_Task(int8_t ms_count);
void Can_Connection_Init(void);
#endif