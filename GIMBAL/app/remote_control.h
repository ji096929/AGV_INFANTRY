#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

#include "dma.h"
#include "usart.h"



#include "stdint.h"
//键鼠控制参数
#define CHASSIS_LOW_SPEED	100
#define CHASSIS_HIGH_SPEED 300

#define KEY_MID 0
#define KEY_MAX 427
#define KEY_MIN -427
#define KEY_DEADLINE	0

#define YAW_MAX_TARGET_ANGLE_NORMAL	9.0f
#define YAW_MIN_TARGET_ANGLE_NORMAL	-9.0f
#define YAW_MAX_TARGET_ANGLE_PRECISION	3.0f
#define YAW_MIN_TARGET_ANGLE_PRECISION	-3.0f
#define YAW_MAX_TARGET_ANGLE_RUNNING	3.0f
#define YAW_MIN_TARGET_ANGLE_RUNNING	-3.0f

#define PITCH_MAX_TARGET_ANGLE_NORMAL	6.0f
#define PITCH_MIN_TARGET_ANGLE_NORMAL	-6.0f
#define PITCH_MAX_TARGET_ANGLE_PRECISION	3.0f
#define PITCH_MIN_TARGET_ANGLE_PRECISION	-3.0f
#define PITCH_MAX_TARGET_ANGLE_RUNNING	3.0f
#define PITCH_MIN_TARGET_ANGLE_RUNNING	-3.0f

//遥控器控制参数
#define DEADLINE 20.0f
#define YAW_MAX_TARGET_ANGLE 3.0f
#define YAW_MIN_TARGET_ANGLE -3.0f
#define YAW_MAX_SPEED 6.0f
#define YAW_MIN_SPEED -6.0f

#define PITCH_MAX_TARGET_ANGLE 3.0f
#define PITCH_MIN_TARGET_ANGLE -3.0f
#define PITCH_MAX_SPEED 6.0f
#define PITCH_MIN_SPEED -6.0f

#define RC_MAX 660.0f
#define RC_MIN -660.0f
#define RC_MID 0.0f

#define	Transmission_Mode_OFF	0
#define Transmission_Mode_ON	1

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */

#define SW_L                    RC.rc_receive.rc.sw[1]
#define SW_R                    RC.rc_receive.rc.sw[0]
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(sw)       (sw == RC_SW_DOWN)
#define switch_is_mid(sw)        (sw == RC_SW_MID)
#define switch_is_up(sw)         (sw == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
#define MOUSE_x               RC.rc_receive.mouse.x     
#define MOUSE_y               RC.rc_receive.mouse.y
#define MOUSE_z               RC.rc_receive.mouse.z
#define MOUSE_pre_left        RC.rc_receive.mouse.press_l
#define MOUSE_pre_right       RC.rc_receive.mouse.press_r
#define KEY_board             RC.rc_receive.key_board.key_code


typedef __packed struct
{
	__packed struct
	{
			int16_t ch[5];
			char sw[2];
	} rc;
	__packed struct
	{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
	} mouse;
	__packed union 
		{
    uint16_t key_code;
   __packed struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } button;
  } key_board;
		
	uint32_t receive_time;
	uint32_t last_receive_time;

} RC_CONTROL_T;



typedef __packed struct{
	
	float target_angle;
	float target_speed;
}GIMBAL_RC_t;

typedef __packed struct{
	GIMBAL_RC_t yaw;
	GIMBAL_RC_t pitch;
	
	float x_speed;
	float y_speed;
	float r_speed;
	
}RC_GET_t;

typedef __packed enum 
{
	HANDLE_CONTROL	=	0X00U,
	KEYBOARD_CONTROL	=	0X01U,
}CONTROL_MODE_T;

typedef __packed struct
{
	RC_CONTROL_T rc_receive;
	RC_GET_t			rc_sent;
	CONTROL_MODE_T	mode;
}RC_T;

typedef __packed struct
{
	uint16_t vision_mode_cnt;
	uint16_t precision_mode_cnt;
	uint16_t fric_mode_cnt;
	uint16_t shoot_number_cnt;
	uint16_t gimbal_cali_cnt;
	uint16_t spin_mode_cnt;
	uint16_t follow_switch_cnt;
	uint16_t	invert_cnt;
}DELAY_TIME_T;

void Remote_Init(void);
extern DELAY_TIME_T delay_time;

extern RC_T RC;
extern int Transmission_Mode;
void Delay_Cnt_Task(void);
void Remote_Task(void);

#endif

