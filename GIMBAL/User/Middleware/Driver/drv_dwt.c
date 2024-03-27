#include "drv_dwt.h"

DWT_Time_t SysTime;
uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
uint32_t CYCCNT_RountCount;
uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;


void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* ʹ��DWT���� */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT�Ĵ���������0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* ʹ��Cortex-M DWT CYCCNT�Ĵ��� */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

