#ifndef KINEMATIC_H_
#define	KINEMATIC_H_

#include "chassis.h"
#include <stdlib.h>

#define wheel_diameter  12.000000f			//ÂÖ×ÓÖ±¾¶
#define half_width    16.0f                           //25.000000f		//µ×ÅÌ°ë¿í
#define half_length   16.0f                           //35.000000f		//µ×ÅÌ°ë³¤

#define PI 			3.141593f
#define PI2     2*PI
#define RPM2RAD 0.104720f										//×ªËÙ×ª½ÇËÙ¶È		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//×ªËÙ×ªÏßËÙ¶È		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//ÏßËÙ¶È×ª×ª¶È
#define M2006_REDUCTION_RATIO 36.000000f		//³ÝÂÖÏä¼õËÙ±È
#define M3508_REDUCTION_RATIO 19.000000f		//³ÝÂÖÏä¼õËÙ±È
#define GM6020_ENCODER_ANGLE  8192.0f


#define MAX_MOTOR_SPEED   400				//µç»ú×î´ó×ªËÙ£¬ºê¶¨Òå·½±ãÐÞ¸Ä   ·¶Î§0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    120.817f    //µ×ÅÌ×î´óÆ½ÒÆËÙ¶È£¬µ¥Î»cm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f    //µ×ÅÌ×î´óÐý×ªËÙ¶È£¬µ¥Î»rad/s    
#define NORMAL_LINEAR_SPEED          70.0f
#define NORMAL_ROTATIONAL_SPEED      0.5f

void Chassis_Speed_Control(CHASSIS_T *chassis);

#endif