#include "chassis_task.h"

#define Super_Cap_RX_Typecode 12
#define SuperCap_Power_TX_Typecode 13
#define SuperCap_KeepAlive_TX_Typecode 14
#define SuperCap_Status_RX_Typecode 15

void Supercap_Uart_Init(void);
void Supercap_Task(void);
void Tx_Super_Capacitor(void);
uint8_t supercap_rx_buffer[11];