#ifndef GIMBAL_CONNECTION_H_
#define GIMBAL_CONNECTION_H_

#include "drv_can.h"
#include "chassis.h"
typedef struct 
{
   float remp;
}GIMBAL_TX_T;

typedef struct 
{
   CHASSIS_MODE_E mode;
   bool invert_flag;
   bool follow_flag;
   float  vx;
   float vy;
   float vw;

}GIMBAL_RX_T;

typedef struct 
{
    GIMBAL_RX_T connection_rx;
    GIMBAL_TX_T connection_tx;

}GIMBAL_CONNECTION_T;

extern GIMBAL_CONNECTION_T connection;
#endif