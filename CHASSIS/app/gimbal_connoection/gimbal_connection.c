#include "gimbal_connection.h"

GIMBAL_CONNECTION_T connection;

void Chassis_Speed_Command_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				connection->connection_rx.vx = (int16_t)(data[0]<<8|data[1]);
        connection->connection_rx.vy = (int16_t)(data[2]<<8|data[3]);
        connection->connection_rx.vw = (int16_t)(data[4]<<8|data[5]);	
}

void Chassis_Control_Mode_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				connection->connection_rx.mode = data[0];
        connection->connection_rx.invert.flag = data[1];
        connection->connection_rx.follow.flag = data[2];
}
