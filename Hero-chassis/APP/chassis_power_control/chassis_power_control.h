#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H


/* USER Settings -------------------------------------------------------------*/
#include <stdint.h>
#define  DEFAULT_SET_VECTOR_SPEED_ONLY_CNT 1 /* 默认仅设置速度的次数 */
#define VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY -114514
typedef struct
{

  uint16_t power_limit_max;     // 功率限制最大值

  uint16_t power_limit_default; // 功率限制默认值

  uint16_t power_limit_flag;    // 功率限制标志

  uint8_t set_vector_speed_only_cnt;
  float scaled_power_coefficient_32; // 功率缩放系数
  uint8_t all_mscb_ready_flag;
  float expect_power_32[4];
  float scaled_power_32[4];
    }   chassis_power_control_t;

    extern chassis_power_control_t chassis_power_control;
void calculate_true_power(void);
  //  void Chassis_Power_Control_Init(void);

#endif