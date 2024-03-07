#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include "Briter_Encoder.h"

#define DMA_REC_LEN    100

#define Briter_Encoder1_TypeCode 1
#define Briter_Encoder2_TypeCode 2	
#define Briter_Encoder3_TypeCode 3
#define Briter_Encoder4_TypeCode 4	

#define Chassis_Motor_Current_TypeCode  5
#define Shoot_Info_TypeCode 			6
#define Yaw_Data_Trans_Typecode  		7
#define YAW_CONTROL_TypeCode 	  		8
#define WheelVel_TypeCode 				9
#define Trigger_TypeCode 				11
#define Super_Cap_RX_Typecode 			12
#define SuperCap_Power_TX_Typecode		13
#define SuperCap_KeepAlive_TX_Typecode	14
#define SuperCap_Status_RX_Typecode		15
#define SuperCap_ErrorCode_RX_Typecode	41

#define Yaw_Control_Current_Mode	   1
#define Yaw_Control_Target_Angle_Mode  2
#define Yaw_Control_Add_Angle_Mode	   0

extern uint8_t UART1_Sent_Data[10];
extern uint8_t UART1_Data_Type;
extern uint8_t dma_rx_buff[20];
void uart_init(void);
void DMA_Send(void);
void UartTX_trigger(void);
void UartTX_To_BetaBoard_Briter_Encoder(Briter_Encoder_t Encoder,int Typecode);
void UartTX_To_BetaBoard_Yaw_Control(int Control_Mode, float Value);
void UartTX_To_BetaBoard_WheelVel(float Speed1, float Speed2, float Speed3, float Speed4);
#endif

