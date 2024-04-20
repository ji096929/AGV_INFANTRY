#ifndef KINEMATIC_H_
#define KINEMATIC_H_

#include "chassis.h"
#include <stdlib.h>

#define wheel_diameter 12.000000f   // ÂÖ×ÓÖ±¾¶
#define half_width 15.95f           // 25.000000f		x方向为宽
#define half_length 16.0f           // 35.000000f
#define ROTATION_CENTER_OFFSET 0.0f // 旋转中心位置偏移量，现在只有y方向上的偏移，且向y负方向偏移，这个偏移量为绝对值

#define THETA_A atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_B atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_C atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_D atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）

#define R_A half_width / cos(THETA_A) // 旋转中心与A舵轮的距离
#define R_B half_width / cos(THETA_B) // 旋转中心与B舵轮的距离
#define R_C half_width / cos(THETA_C) // 旋转中心与C舵轮的距离
#define R_D half_width / cos(THETA_D) // 旋转中心与D舵轮的距离

#define PI 3.141593f
#define PI2 2 * PI
#define RPM2RAD 0.104720f                // ×ªËÙ×ª½ÇËÙ¶È		1 rpm = 2pi/60 rad/s
#define RPM2VEL 0.523599f                // ×ªËÙ×ªÏßËÙ¶È		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f                // ÏßËÙ¶È×ª×ª¶È
#define M2006_REDUCTION_RATIO 36.000000f // ³ÝÂÖÏä¼õËÙ±È
#define M3508_REDUCTION_RATIO 19.000000f // ³ÝÂÖÏä¼õËÙ±È
#define GM6020_ENCODER_ANGLE 8192.0f

#define MAX_MOTOR_SPEED 500                 // µç»ú×î´ó×ªËÙ£¬ºê¶¨Òå·½±ãÐÞ¸Ä   ·¶Î§0 - 10000   15336
#define MAX_BASE_LINEAR_SPEED 120.817f      // µ×ÅÌ×î´óÆ½ÒÆËÙ¶È£¬µ¥Î»cm/s
#define MAX_BASE_ROTATIONAL_SPEED 7.260570f // µ×ÅÌ×î´óÐý×ªËÙ¶È£¬µ¥Î»rad/s
#define NORMAL_LINEAR_SPEED 70.0f
#define NORMAL_ROTATIONAL_SPEED 0.5f

#define RAD_TO_8191 8191.0f / PI / 2
void Chassis_Speed_Control(CHASSIS_T *chassis);

#endif