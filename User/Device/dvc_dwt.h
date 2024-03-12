#ifndef DVC_DWT_H
#define DVC_DWT_H

#include "drv_dwt.h"

#define uint32_max 4294967295u

float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

extern DWT_Time_t SysTime;
extern uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
extern uint32_t CYCCNT_RountCount,CYCCNT_LAST,CYCCNT64;
#endif
