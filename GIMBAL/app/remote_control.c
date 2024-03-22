#include "remote_control.h"
#include "stdarg.h"
#include "math.h"
#include "gimbal.h"
#include "fric.h"
#include "vision.h"
#include "can_connection.h"
#include "trigger.h"
#define RC_huart    huart3
#define RC_UART		USART3
#define RC_dma		hdma_usart3_rx

int Transmission_Mode = Transmission_Mode_OFF;

int16_t int16_abs(int16_t a)
{
	if(a<0) return -a;
	else return a;
}

DELAY_TIME_T delay_time;
RC_T RC;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_CONTROL_T *rc_ctrl);
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
static void RC_restart(uint16_t dma_buf_num);


void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}




static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	
	SET_BIT(RC_huart.Instance->CR3, USART_CR3_DMAR);
	
	__HAL_UART_ENABLE_IT(&RC_huart, UART_IT_IDLE);
	
	__HAL_DMA_DISABLE(&RC_dma);

	while(RC_dma.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&RC_dma);
	}

	RC_dma.Instance->PAR = (uint32_t) & (RC_UART->DR);
	
	RC_dma.Instance->M0AR = (uint32_t)(rx1_buf);
	
	RC_dma.Instance->M1AR = (uint32_t)(rx2_buf);
	
	RC_dma.Instance->NDTR = dma_buf_num;
	
	SET_BIT(RC_dma.Instance->CR, DMA_SxCR_DBM);
	
	__HAL_DMA_ENABLE(&RC_dma);
}



uint8_t RC_data_is_error(void)
{
    
    if (int16_abs(RC.rc_receive.rc.ch[0]) > 660)
    {
        goto error;
    }
    if (int16_abs(RC.rc_receive.rc.ch[1]) > 660)
    {
        goto error;
    }
    if (int16_abs(RC.rc_receive.rc.ch[2]) > 660)
    {
        goto error;
    }
    if (int16_abs(RC.rc_receive.rc.ch[3]) > 660)
    {
        goto error;                      
    }
    if (RC.rc_receive.rc.sw[0] == 0)
    {
        goto error;
    }
    if (RC.rc_receive.rc.sw[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    memset(&RC.rc_receive,0,sizeof(RC.rc_receive));
    return 1;
}


void USART3_IRQHandler(void)
{
	if(RC_huart.Instance->SR & UART_FLAG_RXNE)//���յ�����
	{
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
	}
	else if(RC_UART->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
		if ((RC_dma.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */
			//disable DMA
			//ʧЧ DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//�����趨���ݳ���
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 1
			//�趨������ 1
			RC_dma.Instance->CR |= DMA_SxCR_CT;
			//enable DMA
			//ʹ�� DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				sbus_to_rc(sbus_rx_buf[0], &RC.rc_receive);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//ʧЧ DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//�����趨���ݳ���
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 0
			//�趨������ 0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			//enable DMA
			//ʹ�� DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//����ң��������
				sbus_to_rc(sbus_rx_buf[1], &RC.rc_receive);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
	}
}


static void RC_restart(uint16_t dma_buf_num)
{
	
	__HAL_UART_DISABLE(&RC_huart);
	
	__HAL_DMA_DISABLE(&RC_dma);
	
	RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
	__HAL_UART_CLEAR_IDLEFLAG(&RC_huart);
	__HAL_DMA_CLEAR_FLAG(&RC_dma,DMA_FLAG_TCIF2_6);
	
	__HAL_UART_ENABLE(&RC_huart);
	
	__HAL_DMA_ENABLE(&RC_dma);
}



static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_CONTROL_T *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
			return ;
	}
	
	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; 
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; 
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; 
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); 
	
	rc_ctrl->rc.ch[0] -= 1024;
	rc_ctrl->rc.ch[1] -= 1024;
	rc_ctrl->rc.ch[2] -= 1024;
	rc_ctrl->rc.ch[3] -= 1024;
	rc_ctrl->rc.ch[4] -= 1024;

	if(abs(rc_ctrl->rc.ch[0])<5)
		rc_ctrl->rc.ch[0]	=	0;
	if(abs(rc_ctrl->rc.ch[1])<5)
		rc_ctrl->rc.ch[1]	=	0;
	if(abs(rc_ctrl->rc.ch[2])<5)
		rc_ctrl->rc.ch[2]	=	0;
	if(abs(rc_ctrl->rc.ch[3])<5)
		rc_ctrl->rc.ch[3]	=	0;
	
	rc_ctrl->rc.sw[0] = ((sbus_buf[5] >> 4) & 0x0003); 
	rc_ctrl->rc.sw[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; 

	if (Transmission_Mode == Transmission_Mode_OFF)
	{
		rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); 
		rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); 
		rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
		rc_ctrl->mouse.press_l = sbus_buf[12]; 
		rc_ctrl->mouse.press_r = sbus_buf[13]; 
		rc_ctrl->key_board.key_code = sbus_buf[14] | (sbus_buf[15] << 8); 
	}	
	
		rc_ctrl->receive_time++;
	
}

//接收值数据映射
float rc_control_cali(float max_out, float min_out, float actual, float max_in, float mid_in, float min_in, float deadline)
{
    float out;
    if (actual - mid_in > deadline)
    {
        out = (actual - mid_in - deadline) / (max_in - mid_in);
        out *= max_out;
    }
    else if (actual - mid_in < -deadline)
    {
        out = (mid_in - actual - deadline) / (mid_in - min_in);
        out *= min_out;
    }
    else
    {
        out = 0;
    }
    
    return out;
}


//遥控器状态更新
void Rc_Mode_Update(void)
{
	if(RC.rc_receive.rc.ch[0]!=0||RC.rc_receive.rc.ch[1]!=0||RC.rc_receive.rc.ch[2]!=0||RC.rc_receive.rc.ch[3]!=0)
		RC.mode=HANDLE_CONTROL;
	if(RC.rc_receive.key_board.key_code||RC.rc_receive.mouse.x||RC.rc_receive.mouse.y||RC.rc_receive.mouse.z)
		RC.mode=KEYBOARD_CONTROL;
};

void Control_Mode_Update(void)
{
	switch(RC.mode)
	{
		case HANDLE_CONTROL :
		//摇杆控制
		//无力状态
			if (switch_is_down(SW_L)&&switch_is_down(SW_R)&&gimbal.parameter.mode != GIMBAL_MODE_NO_FORCE)
			{
				gimbal.parameter.mode = GIMBAL_MODE_NO_FORCE;
				chassis.send.mode = CHASSIS_MODE_NOFORCE;
			}
		//随动状态
			if (switch_is_down(SW_L)&&switch_is_mid(SW_R)&&gimbal.parameter.mode != GIMBAL_MODE_ABSOLUTE)
			{
				gimbal.parameter.mode = GIMBAL_MODE_ABSOLUTE;
				chassis.send.mode = CHASSIS_MODE_ABSOLUTE;

			}
		//小陀螺状态
			if (switch_is_down(SW_L)&&switch_is_up(SW_R)&&gimbal.parameter.mode != GIMBAL_MODE_TOPANGLE)
			{
				gimbal.parameter.mode = GIMBAL_MODE_TOPANGLE;
				chassis.send.mode = CHASSIS_MODE_TOPANGLE;

			}
			
			break;
		case KEYBOARD_CONTROL :
			//键鼠控制
			//小陀螺模式切换
		if(RC.rc_receive.key_board.button.CTRL&&RC.rc_receive.key_board.button.R)
		{
			if (delay_time.spin_mode_cnt	==	0&&gimbal.parameter.mode != GIMBAL_MODE_TOPANGLE)
			{
				gimbal.parameter.mode = GIMBAL_MODE_TOPANGLE;
				chassis.send.mode = CHASSIS_MODE_TOPANGLE;
				delay_time.spin_mode_cnt	=400;

			}
			if (delay_time.spin_mode_cnt	==	0&&gimbal.parameter.mode == GIMBAL_MODE_TOPANGLE)
			{
				gimbal.parameter.mode = GIMBAL_MODE_ABSOLUTE;
				chassis.send.mode = CHASSIS_MODE_ABSOLUTE;
				delay_time.spin_mode_cnt	=400;

			}
		}
				
			
			break;
		
	}
};

void Remote_Command_Update(void)
{
	switch(RC.mode)
	{
		case HANDLE_CONTROL :
			//摇杆控制
			RC.rc_sent.yaw.target_angle=rc_control_cali(YAW_MAX_TARGET_ANGLE,YAW_MIN_TARGET_ANGLE,RC.rc_receive.rc.ch[0],RC_MAX,RC_MID,RC_MIN,DEADLINE);


			RC.rc_sent.pitch.target_angle=rc_control_cali(PITCH_MAX_TARGET_ANGLE,PITCH_MIN_TARGET_ANGLE,RC.rc_receive.rc.ch[1],RC_MAX,RC_MID,RC_MIN,DEADLINE);

			
			RC.rc_sent.x_speed=RC.rc_receive.rc.ch[3]/2.0f;
			RC.rc_sent.y_speed=RC.rc_receive.rc.ch[2]/2.0f;
			RC.rc_sent.r_speed=RC.rc_receive.rc.ch[4]/660.0f*5.0f;

			break;
		case KEYBOARD_CONTROL :
			//键鼠控制
			RC.rc_sent.x_speed	=	0;
			RC.rc_sent.y_speed	=	0;
		if(RC.rc_receive.key_board.button.SHIFT)
		{
			//高速运动
			if(RC.rc_receive.key_board.button.W)
				RC.rc_sent.x_speed	=	CHASSIS_LOW_SPEED;
			if(RC.rc_receive.key_board.button.S)
				RC.rc_sent.x_speed	=	-CHASSIS_LOW_SPEED;
			if(RC.rc_receive.key_board.button.D)
				RC.rc_sent.x_speed	=	CHASSIS_LOW_SPEED;		
			if(RC.rc_receive.key_board.button.A)
				RC.rc_sent.x_speed	=	-CHASSIS_LOW_SPEED;
				switch(gimbal.parameter.mode)
				{
				case	GIMBAL_MODE_PRECISION:
				RC.rc_sent.yaw.target_angle=rc_control_cali(YAW_MAX_TARGET_ANGLE_PRECISION,YAW_MIN_TARGET_ANGLE_PRECISION,RC.rc_receive.mouse.x,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				RC.rc_sent.pitch.target_angle=rc_control_cali(PITCH_MAX_TARGET_ANGLE_PRECISION,PITCH_MIN_TARGET_ANGLE_PRECISION,RC.rc_receive.mouse.y,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				break;
				default	:
				RC.rc_sent.yaw.target_angle=rc_control_cali(YAW_MAX_TARGET_ANGLE_RUNNING,YAW_MIN_TARGET_ANGLE_RUNNING,RC.rc_receive.mouse.x,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				RC.rc_sent.pitch.target_angle=rc_control_cali(PITCH_MAX_TARGET_ANGLE_RUNNING,PITCH_MIN_TARGET_ANGLE_RUNNING,RC.rc_receive.mouse.y,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				break;
				}
			
		}
			
		else	if(!RC.rc_receive.key_board.button.SHIFT)
		{
			//低速运动
			if(RC.rc_receive.key_board.button.W)
				RC.rc_sent.x_speed	=	CHASSIS_HIGH_SPEED;
			if(RC.rc_receive.key_board.button.S)
				RC.rc_sent.x_speed	=	-CHASSIS_HIGH_SPEED;
			if(RC.rc_receive.key_board.button.D)
				RC.rc_sent.x_speed	=	CHASSIS_HIGH_SPEED;		
			if(RC.rc_receive.key_board.button.A)
				RC.rc_sent.x_speed	=	-CHASSIS_HIGH_SPEED;
			switch(gimbal.parameter.mode)
			{
				case	GIMBAL_MODE_PRECISION:
				RC.rc_sent.yaw.target_angle=rc_control_cali(YAW_MAX_TARGET_ANGLE_PRECISION,YAW_MIN_TARGET_ANGLE_PRECISION,RC.rc_receive.mouse.x,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				RC.rc_sent.pitch.target_angle=rc_control_cali(PITCH_MAX_TARGET_ANGLE_PRECISION,PITCH_MIN_TARGET_ANGLE_PRECISION,RC.rc_receive.mouse.y,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				break;
				default	:
				RC.rc_sent.yaw.target_angle=rc_control_cali(YAW_MAX_TARGET_ANGLE_NORMAL,YAW_MIN_TARGET_ANGLE_NORMAL,RC.rc_receive.mouse.x,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				RC.rc_sent.pitch.target_angle=rc_control_cali(PITCH_MAX_TARGET_ANGLE_NORMAL,PITCH_MIN_TARGET_ANGLE_NORMAL,RC.rc_receive.mouse.y,KEY_MAX,KEY_MID,KEY_MIN,KEY_DEADLINE);
				break;
			}
	
		}
		
			break;
		
	}

};

void Vision_Control_Mode_Update(void)
{
		switch(RC.mode)
	{
		case HANDLE_CONTROL :
			
			if(switch_is_up(SW_L)&&switch_is_down(SW_R))
			{
				if(delay_time.vision_mode_cnt==0&&vision_control.mode==VISION_OFF)
				{
					vision_control.mode=VISION_ON;
					delay_time.vision_mode_cnt=400;
				}
				if(delay_time.vision_mode_cnt==0&&vision_control.mode==VISION_ON)
				{
					vision_control.mode=VISION_OFF;
					delay_time.vision_mode_cnt=400;
				}
			}
				
			
		
			break;
		case KEYBOARD_CONTROL :
						if(RC.rc_receive.key_board.button.V)
			{
				if(delay_time.vision_mode_cnt==0&&vision_control.mode==VISION_OFF)
				{
					vision_control.mode=VISION_ON;
					delay_time.vision_mode_cnt=400;
				}
				if(delay_time.vision_mode_cnt==0&&vision_control.mode==VISION_ON)
				{
					vision_control.mode=VISION_OFF;
					delay_time.vision_mode_cnt=400;
				}
			}
			
			break;
		
	}
};

void Fric_Wheel_Mode_Update(void)
{
	switch (RC.mode)
	{
	case HANDLE_CONTROL :
		        if(switch_is_mid(SW_L)&&switch_is_up(SW_R))
				{
					if(delay_time.fric_mode_cnt==0&&fric.parameter.mode!=FRIC_STOP)
					{
						fric.parameter.mode=FRIC_STOP;
						delay_time.fric_mode_cnt=400;
					}
					if(delay_time.fric_mode_cnt==0&&fric.parameter.mode!=FRIC_RUNNING)
					{
						fric.parameter.mode=FRIC_RUNNING;
						delay_time.fric_mode_cnt=400;
					}
				}
				
		break;
	case KEYBOARD_CONTROL :
				
		    if(RC.rc_receive.mouse.press_r)
				{
					if(delay_time.fric_mode_cnt==0&&fric.parameter.mode!=FRIC_STOP)
					{
						fric.parameter.mode=FRIC_STOP;
						delay_time.fric_mode_cnt=400;
					}
					if(delay_time.fric_mode_cnt==0&&fric.parameter.mode!=FRIC_RUNNING)
					{
						fric.parameter.mode=FRIC_RUNNING;
						delay_time.fric_mode_cnt=400;
					}
				}
			break;
	}
};



void Precision_Mode_Update(void)
{
    	switch(RC.mode)
	{
		case HANDLE_CONTROL :
			
			if(switch_is_up(SW_L)&&switch_is_up(SW_R))
			{
				if(delay_time.precision_mode_cnt==0&&gimbal.parameter.mode!=GIMBAL_MODE_PRECISION)
				{
					gimbal.parameter.mode=GIMBAL_MODE_PRECISION;
					chassis.send.mode = CHASSIS_MODE_PRECISE;
					delay_time.precision_mode_cnt=400;
				}
				if(delay_time.precision_mode_cnt==0&&gimbal.parameter.mode==GIMBAL_MODE_PRECISION)
				{
					gimbal.parameter.mode=GIMBAL_MODE_ABSOLUTE;
					chassis.send.mode = CHASSIS_MODE_ABSOLUTE;
					delay_time.precision_mode_cnt=400;
				}
			}
				
			
		
			break;
		case KEYBOARD_CONTROL :
			if(RC.rc_receive.key_board.button.R)
			{
				if(delay_time.precision_mode_cnt==0&&gimbal.parameter.mode!=GIMBAL_MODE_PRECISION)
				{
					gimbal.parameter.mode=GIMBAL_MODE_PRECISION;
					chassis.send.mode = CHASSIS_MODE_PRECISE;
					delay_time.precision_mode_cnt=400;
				}
				if(delay_time.precision_mode_cnt==0&&gimbal.parameter.mode==GIMBAL_MODE_PRECISION)
				{
					gimbal.parameter.mode=GIMBAL_MODE_ABSOLUTE;
					chassis.send.mode = CHASSIS_MODE_ABSOLUTE;
					delay_time.precision_mode_cnt=400;
				}
			}
			break;
		
	}
};

void Trigger_Shoot_Number_Update(void)
{
	switch (RC.mode)
	{
	case HANDLE_CONTROL :
		    switch(fric.parameter.mode)
	{
		case	FRIC_STOP	:
		case	FRIC_ERROR	:
			trigger.parameter.state	=	TRIGGER_STOP;
		
			break;
		
		case FRIC_RUNNING	:
			if(switch_is_mid(SW_L)&&switch_is_down(SW_R)	&&	delay_time.shoot_number_cnt	==	0)
			{
				trigger.parameter.state	=	TRIGGER_RUNNING;
				delay_time.shoot_number_cnt=400;
				trigger.parameter.shoot_num++;
			}
			break;
	}
				
		break;
	case KEYBOARD_CONTROL :
		switch(fric.parameter.mode)
	{
		case	FRIC_STOP	:
		case	FRIC_ERROR	:
			trigger.parameter.state	=	TRIGGER_STOP;
		
			break;
		
		case FRIC_RUNNING	:
			if(RC.rc_receive.mouse.press_l	&&	delay_time.shoot_number_cnt	==	0)
			{
				trigger.parameter.state	=	TRIGGER_RUNNING;
				delay_time.shoot_number_cnt=400;
				trigger.parameter.shoot_num++;
			}
			break;
	}
			break;
	}
}



void Follow_Switch_Flag_Update(void)
{
	switch(RC.mode)
	{
		case HANDLE_CONTROL :
		
			break;
		case KEYBOARD_CONTROL :
		if(RC.rc_receive.key_board.button.G	)
	{
		if(gimbal.parameter.mode	==	GIMBAL_MODE_ABSOLUTE	&&	chassis.send.follow_flag	==	0	&&delay_time.follow_switch_cnt	==	0)
		{
			delay_time.follow_switch_cnt	=	400;
			chassis.send.follow_flag	=1	;
		}
		if(gimbal.parameter.mode	==	GIMBAL_MODE_ABSOLUTE	&&	chassis.send.follow_flag	==	1	&&delay_time.follow_switch_cnt	==	0)
		{
			delay_time.follow_switch_cnt	=	400;
			chassis.send.follow_flag	=0	;
		}
	}
			break;
		
	}

};

void Invert_Flag_Update(void)
{
	switch(RC.mode)
	{
		case HANDLE_CONTROL :
		
			break;
		case KEYBOARD_CONTROL :
			if(RC.rc_receive.key_board.button.C	)
	{
		if(gimbal.parameter.mode	==	GIMBAL_MODE_ABSOLUTE	&&	chassis.send.invert_flag	==	0	&&delay_time.invert_cnt	==	0)
		{
			delay_time.invert_cnt	=	400;
			chassis.send.invert_flag	=1	;
		}
		if(gimbal.parameter.mode	==	GIMBAL_MODE_ABSOLUTE	&&	chassis.send.invert_flag	==	1	&&delay_time.invert_cnt	==	0)
		{
			delay_time.invert_cnt	=	400;
			chassis.send.invert_flag	=0	;
		}
	}
			break;
		
	}

};

void Delay_Cnt_Task(void)
{
	//vision_mode_cnt--;
	if(delay_time.vision_mode_cnt) delay_time.vision_mode_cnt--;
	//precision_mode_cnt--;
	if(delay_time.precision_mode_cnt) delay_time.precision_mode_cnt--;
	//fric_mode_cnt--;
	if(delay_time.fric_mode_cnt) delay_time.fric_mode_cnt--;
	//shoot_number_cnt--;
	if(delay_time.shoot_number_cnt) delay_time.shoot_number_cnt--;
	//spin_mode_cnt--;
	if(delay_time.spin_mode_cnt)	delay_time.spin_mode_cnt--;
	//invert_cnt--;
	if(delay_time.invert_cnt)	delay_time.invert_cnt--;
	//follow_switch_cnt--;
	if(delay_time.follow_switch_cnt)	delay_time.invert_cnt--;
};

void Remote_Init(void)
{
	//初始化遥控器变量
	memset(&RC,0,sizeof(RC));
	RC.mode=HANDLE_CONTROL;
	//遥控器内容赋值
	remote_control_init();
}

void Remote_Task(void)
{
	//吊射模式状态更新
	Precision_Mode_Update();
	//射击数量更新
	Trigger_Shoot_Number_Update();
	//底盘随动状态更新
	Follow_Switch_Flag_Update();
	//更新反转标志状态
	Invert_Flag_Update();
	//更新摩擦轮模式状态
	Fric_Wheel_Mode_Update();
	//更新视觉控制模式状态
	Vision_Control_Mode_Update();
	//更新遥控指令控制值
	Remote_Command_Update();
	//更新控制模式
	Control_Mode_Update();
	//更新遥控器模式
	Rc_Mode_Update();
};

