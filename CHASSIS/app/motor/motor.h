/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-09 21:46:49
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 15:55:34
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\motor\motor.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef MOTOR_H_
#define MOTOR_H_

#define PI 3.141593f

#include "stdint.h"
#include "stdbool.h"
typedef struct M3508_STATUS_T
{
    float given_current;
    float given_torque;
    int16_t velocity_rpm;
    int16_t temperature;

    uint16_t position;
    uint16_t last_position;
    float    position_rad;
    float    position_degree;
    int16_t  position_rounds;
    int32_t  total_position;
    float    total_position_rad;
    float    total_position_degree;
    
}M3508_STATUS_T;


typedef struct M3508_FEEDBACK_T
{
    int16_t given_current_lsb;
    uint16_t position_lsb;
    int16_t velocity_lsb;
    int16_t temperature_lsb; 
}M3508_FEEDBACK_T;

typedef struct M3508_COMMAND_T
{
    float give_current;
    int16_t give_current_lsb;
    float give_torque;

}M3508_COMMAND_T;


typedef struct  M3508_PARAMETER_T
{
   uint8_t ID;
   float   reduction_rate;
   float   torque_coefficient;
}M3508_PARAMETER_T;


typedef struct M3508_T
{
    M3508_PARAMETER_T parameter;
    M3508_STATUS_T status;
    M3508_FEEDBACK_T feedback;
    M3508_COMMAND_T command;
}M3508_T;


typedef enum
{
    NO_CALIBRATE = 0,
    CALIBRATED = 1,
}GM6020_CALIBRATE_E;


typedef struct GM_6020_STATUS_T
{
    float given_current;
    float given_torque;

    int16_t velocity_rpm;
    int16_t temperature;

    uint16_t position;
    uint16_t last_position;
    float    position_rad;
    float    position_degree;
    int16_t  position_rounds;
    int32_t  total_position;
    float    total_position_rad;
    float    total_position_degree;

    int32_t  last_total_position;
    float    last_total_position_rad;
    float    last_total_position_degree;

    uint16_t zero_position;
    float    zero_position_rad;
    float    zero_position_degree;

}GM_6020_STATUS_T;

typedef struct GM_6020_FEEDBACK_T
{
    int16_t given_current_lsb;
    int16_t velocity_lsb;
    int16_t temperature_lsb;
    int16_t position_lsb;
}GM_6020_FEEDBACK_T;

typedef struct GM_6020_COMMAND_T
{
    int16_t give_voltage_lsb;

}GM_6020_COMMAND_T;

typedef struct GM_6020_PARAMETER_T
{
    uint8_t ID;
    float torque_coefficient;
    GM6020_CALIBRATE_E  calibrate_state;

}GM_6020_PARAMETER_T;

typedef struct GM6020_T
{
    GM_6020_COMMAND_T   command;
    GM_6020_FEEDBACK_T  feedback;
    GM_6020_PARAMETER_T  parameter;
    GM_6020_STATUS_T   status;
}GM6020_T;

 void GM6020_Status_Update(GM6020_T *motor);
  void GM6020_Feedback_Update(GM6020_T *motor,uint8_t rx_data[]);
   void GM6020_Command_Update(GM6020_T *motor,int16_t voltage);
    void GM6020_Init(GM6020_T *motor,uint8_t ID,float torque_coefficient);

 void M3508_Status_Update(M3508_T *motor);
  void M3508_Feedback_Update(M3508_T *motor,uint8_t rx_data[]);
   void M3508_Command_Update(M3508_T *motor,float current);
    void M3508_Init(M3508_T *motor,uint8_t ID,float reduction_rate, float torque_coefficient);

#endif