#include "gimbal_connection.h"

GIMBAL_CONNECTION_T connection;

void Chassis_Flag_Update(GIMBAL_CONNECTION_T *connection)
{
	connection->connection_rx.invert.last_flag=connection->connection_rx.invert.flag;
	connection->connection_rx.follow.last_flag=connection->connection_rx.follow.flag;
	connection->connection_rx.fric.last_flag=connection->connection_rx.fric.flag;
	connection->connection_rx.vision.last_flag=connection->connection_rx.vision.flag;
	connection->connection_rx.Graphic_Init.last_flag=connection->connection_rx.Graphic_Init.flag;
}

void Chassis_Speed_Command_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
	memcpy(&connection->connection_rx.vx, data, sizeof(int16_t));
	memcpy(&connection->connection_rx.vy, data+2, sizeof(int16_t));
	memcpy(&connection->connection_rx.vw, data+4, sizeof(int16_t));
	
//	connection->connection_rx.vx = (int16_t)(data[0]<<8|data[1]);
//        connection->connection_rx.vy = (int16_t)(data[2]<<8|data[3]);
//        connection->connection_rx.vw = (int16_t)(data[4]<<8|data[5]);	
}

void Chassis_Control_Mode_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				
				connection->connection_rx.mode = data[0];
        connection->connection_rx.invert.flag = data[1];
        //connection->connection_rx.follow.flag = data[2];
	        connection->connection_rx.follow.flag = 1;
				connection->connection_rx.fric.flag	=	data[3];
				connection->connection_rx.vision.flag	=	data[4];
				connection->connection_rx.Graphic_Init.flag=data[5];
}

void Fric_Speed_And_Pitch_Angle_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[])
{
				memcpy(&connection->connection_rx.fric_speed,&data[0],2);
				memcpy(&connection->connection_rx.pitch_angle,&data[2],4);
}

