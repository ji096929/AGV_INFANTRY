#ifndef AGV_CONTROL_H_
#define AGV_CONTROL_H_

#include <stdint.h>
#include "drv_can.h"
#include "pid.h"
#define  DEFAULT_SET_VECTOR_SPEED_ONLY_CNT 1 /* ?????????? */
#define VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY -114514
typedef struct
{

  uint16_t power_limit_max;     // ???????

  uint16_t power_limit_default; // ???????

  uint16_t power_limit_flag;    // ??????

  uint8_t set_vector_speed_only_cnt;
  float scaled_power_coefficient_32; // ??????
  uint8_t all_mscb_ready_flag;
  float expect_power_32[4];
  float scaled_power_32[4];
    }   chassis_power_control_t;

		extern chassis_power_control_t chassis_power_control;
void calculate_true_power(void);
		void AGV_connoection(int ms_cnt);
		void Chassis_Power_Control_Init(void);

 extern PID_TypeDef buffer_pid;
   extern  PID_TypeDef supercap_pid;

#endif