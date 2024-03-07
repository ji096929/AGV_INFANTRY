#ifndef __BRITER_ENCODER_H
#define __BRITER_ENCODER_H

#include <stdint.h>

// 宏定义设置

#define Briter_Auto_Callback_Mode 0
#define Briter_Request_Mode       1

// 布瑞特编码器的结构体
typedef struct
{
    uint8_t Length;
	uint8_t ID;
    uint8_t Order_Code;
    uint8_t Send_Data[7];
    uint8_t Received_Data[7];
    int Last_Angle;
    int Raw_Present_Angle;
    int Absolute_Angle;
    int Total_Angle;
    int Raw_Round;
    int Total_Round;
    float Angle;
    float Angular_Speed;
	int Request_Flag;
}Briter_Encoder_t;

extern Briter_Encoder_t Briter_Encoder1, Briter_Encoder2, Briter_Encoder3, Briter_Encoder4;

void Send_Data_Initialize(void);
void Briter_Send_Data(int ID);
void Init_Encoder_Struct(void);
void Get_Encoder_Position(int ID);
int All_Initial_Position_Recorded(void);

void Briter_Encoder_Task(int S_Cnt, int MS_Cnt);

#endif
