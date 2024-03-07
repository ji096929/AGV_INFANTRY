#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

/* USER Settings -------------------------------------------------------------*/
#include <stdint.h>
#define DEFAULT_SET_VECTOR_SPEED_ONLY_CNT 1 /* 默认仅设置速度的次数 */
#define VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY -114514
typedef struct
#define TOQUE_COEFFICIENT 2.7390496e-6 // (20/16384)*(0.3)*(1/14)/9.55
#define K1  1//1为默认数值
#define K2 1 // 1为默认数值
#define CONSTANT  1//功率模型中的常数
{

    uint16_t power_limit_max;     // 功率限制最大值
    uint16_t power_limit_min;     // 功率限制最小值
    uint16_t power_limit_step;    // 功率限制步长
    uint16_t power_limit_default; // 功率限制默认值
    uint16_t power_limit_current; // 功率限制当前值
    uint16_t power_limit_flag;    // 功率限制标志

    uint8_t set_vector_speed_only_cnt;
    float scaled_power_coefficient_32; // 功率缩放系数
    uint8_t all_mscb_ready_flag;
    float expect_power_32;
    float scaled_power_32;
    uint8_t motor_control_flag;//当该标志位为1时，将电流值发送给电机
} chassis_power_control_t;

extern chassis_power_control_t chassis_power_control;

float calculate_torque_current_according_to_scaled_power(float scaled_power);
//  void Chassis_Power_Control_Init(void);

#endif