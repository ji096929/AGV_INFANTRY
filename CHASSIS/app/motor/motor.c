/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-09 21:46:39
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 03:07:58
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\motor\motor.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "motor.h"

/**********************************M3508 Part*********************************************/

int16_t M3508_Current_From_Ampere_To_Lsb(float current)
{
    return (int16_t)(current /20.0f * 16384.0f);
}

float M3508_Current_From_Lsb_To_Ampere(int16_t current)
{
    return (float)(current /16384.0f*20.0f);
}

void M3508_Init(M3508_T *motor,uint8_t ID,float reduction_rate, float torque_coefficient)
{
    motor->parameter.ID     =   ID;
    motor->parameter.reduction_rate =   reduction_rate;
    motor->parameter.torque_coefficient =  torque_coefficient;
}

void M3508_Command_Update(M3508_T *motor,float current)
{
    motor->command.give_current = current;
    motor->command.give_current_lsb = M3508_Current_From_Ampere_To_Lsb(motor->command.give_current);
    motor->command.give_torque = motor->command.give_current * motor->parameter.reduction_rate*motor->parameter.torque_coefficient;

}

void M3508_Feedback_Update(M3508_T *motor,uint8_t rx_data[])
{
    motor->feedback.position_lsb   =   (uint16_t)(rx_data[0]<<8 | rx_data[1]);
    motor->feedback.velocity_lsb   =   (int16_t)(rx_data[2]<<8 | rx_data[3]);
    motor->feedback.given_current_lsb   =   (int16_t)(rx_data[4]<<8 | rx_data[5]);
    motor->feedback.temperature_lsb   =   rx_data[6];
}

float M3508_Position_From_Lsb_To_Degree(uint16_t Lsb)
{
    return (float)(Lsb / 8192.0f * 360.0f);
}

float M3508_Position_From_Lsb_To_Rad(uint16_t Lsb)
{
    return (float)(Lsb / 8192.0f * 2.0f*PI);
}

void M3508_Status_Update(M3508_T *motor)
{
    motor->status.given_current =   M3508_Current_From_Lsb_To_Ampere(motor->feedback.given_current_lsb);
    motor->status.given_torque  =   motor->parameter.reduction_rate*motor->status.given_current*motor->parameter.torque_coefficient;
    motor->status.velocity_rpm  =   motor->feedback.velocity_lsb;
    motor->status.temperature   =   motor->feedback.temperature_lsb;

    motor->status.last_position =   motor->status.position;
    motor->status.position       =   motor->feedback.position_lsb;
    motor->status.position_degree   =   M3508_Position_From_Lsb_To_Degree(motor->status.position);
    motor->status.position_rad   =   M3508_Position_From_Lsb_To_Rad(motor->status.position);
    if(motor->status.last_position - motor->status.position>4096)
    {
        motor->status.position_rounds++;
    }
    if(motor->status.last_position - motor->status.position<-4096)
    {
        motor->status.position_rounds--;
    }
    motor->status.total_position    =   motor->status.position_rounds*8191+motor->status.position;
    motor->status.total_position_rad =   motor->status.position_rounds*2.0f*PI+motor->status.position_rad;
    motor->status.total_position_degree =   motor->status.position_rounds*360.0f+motor->status.position_degree;
}

/*******************************GM6020 Part********************************************/

 void GM6020_Init(GM6020_T *motor,uint8_t ID,float torque_coefficient)
 {
    motor->parameter.ID =   ID;
    motor->parameter.torque_coefficient =   torque_coefficient;
    motor->parameter.calibrate_state    =   NO_CALIBRATE;

 }

 void GM6020_Command_Update(GM6020_T *motor,int16_t voltage)
 {
    motor->command.give_voltage_lsb =   voltage;
 }

 void GM6020_Feedback_Update(GM6020_T *motor,uint8_t rx_data[])
 {
    motor->feedback.position_lsb    =   (rx_data[0]<<8)|rx_data[1];
    motor->feedback.velocity_lsb    =   (rx_data[2]<<8)|rx_data[3];
    motor->feedback.given_current_lsb    =   (rx_data[4]<<8)|rx_data[5];
    motor->feedback.temperature_lsb    =   rx_data[6];
 }

float GM6020_Current_From_Lsb_To_Ampere(int16_t lsb)
{
    return (float)(lsb/8191.0f*3.0f);
}

float GM6020_Position_From_Lsb_To_Rad(uint16_t lsb)
{
    return (float)(lsb/8191.0f*2.0f*PI);
}

float GM6020_Position_From_Lsb_To_Degree(uint16_t lsb)
{
    return (float)(lsb/8191.0f*360.0f);
}

 void GM6020_Status_Update(GM6020_T *motor)
 {
     motor->status.given_current    =   GM6020_Current_From_Lsb_To_Ampere(motor->feedback.given_current_lsb);
     motor->status.given_torque =   motor->status.given_current*motor->parameter.torque_coefficient;
     motor->status.temperature    =   motor->feedback.temperature_lsb;
     motor->status.velocity_rpm =   motor->feedback.velocity_lsb;

     motor->status.last_position    =   motor->status.position;
     motor->status.position        =   motor->feedback.position_lsb;
     motor->status.position_degree    =   GM6020_Position_From_Lsb_To_Degree(motor->status.position);
     motor->status.position_rad    =   GM6020_Position_From_Lsb_To_Rad(motor->status.position);
    if(motor->status.last_position - motor->status.position>4096)
    {
        motor->status.position_rounds++;
    }
    if(motor->status.last_position - motor->status.position<-4096)
    {
        motor->status.position_rounds--;
    }
    motor->status.last_total_position =   motor->status.total_position;
    motor->status.last_total_position_rad =   motor->status.total_position_rad;
    motor->status.last_total_position_degree =   motor->status.total_position_degree;
    motor->status.total_position    =   motor->status.position_rounds*8191+motor->status.position  -   motor->status.zero_position;
    motor->status.total_position_degree =   motor->status.position_rounds*360.0f+motor->status.position_degree-motor->status.zero_position_degree;
    motor->status.total_position_rad    =   motor->status.position_rounds*2.0f*PI+motor->status.position_rad-motor->status.zero_position_rad;
    if(motor->parameter.calibrate_state==   NO_CALIBRATE)
    {
        motor->status.zero_position    =   motor->status.total_position;
        motor->status.zero_position_degree =   motor->status.total_position_degree;
        motor->status.zero_position_rad    =   motor->status.total_position_rad;
    }
 }