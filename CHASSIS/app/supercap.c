#include "supercap.h"
#include "tim.h"
#include "usart.h"
#include "chassis_task.h"
#include "gimbal_connection.h"
uint8_t CAN2_0x66_Tx_Data[8];
// void Supercap_Uart_Init(void)
//{
//     HAL_UART_Receive_IT(&huart1, supercap_rx_buffer, 11);
// }

// void UART_TX_Supercap_Connection_Check(void)
//{
//     uint8_t Sent_Data[11];
//     int Keep_Alive_Typecode = SuperCap_KeepAlive_TX_Typecode;
//     Sent_Data[0] = '*';
//     Sent_Data[1] = Keep_Alive_Typecode;
//     Sent_Data[2] = (uint8_t)(time.total_count >> 24);
//     Sent_Data[3] = (uint8_t)((time.total_count >> 16) & 0xff);
//     Sent_Data[4] = (uint8_t)((time.total_count >> 8) & 0xff);
//     Sent_Data[5] = (uint8_t)(time.total_count & 0xff);
//     Sent_Data[6] = 0x05;
//     Sent_Data[7] = 0x06;
//     Sent_Data[8] = 0x07;
//     Sent_Data[9] = 0x08;
//     Sent_Data[10] = ';';

//    for (int i = 0; i <= 3; i++)
//        chassis.supercap.KeepAlive_SentData[i] = Sent_Data[i + 2];
//    uint8_t status;
//    status = HAL_UART_Transmit(&huart1, Sent_Data, 11, 0xff);
//}

void Supercap_Keep_Alive(void)
{
    //    chassis.supercap.Keep_Alive_Time_Cnt++;
    //

    //    if (chassis.supercap.Keep_Alive_Time_Cnt > 10)
    //        chassis.supercap.online_state = SUPERCAP_OFFLINE;
    //    else
    //        chassis.supercap.online_state = SUPERCAP_ONLINE;

    chassis.supercap.online_state = SUPERCAP_ONLINE;
}

float Max_Power;
float Add_Power;

void Tx_Super_Capacitor(void)
{
    if (JudgeReceive.remainEnergy > 10.f)
    {
        Add_Power = (JudgeReceive.remainEnergy - 10.f) / 50.f * 60.f;
    }
    else
        Add_Power = 0.f;
    // Max_Power=40.0f;
    Max_Power = JudgeReceive.MaxPower+Add_Power;
    memcpy(&CAN2_0x66_Tx_Data, &Max_Power, 4);
    memcpy(&CAN2_0x66_Tx_Data[4], &JudgeReceive.realChassispower, 4);
    CAN_Send_Data(&hcan2, 0x66, CAN2_0x66_Tx_Data, 8);
}
void Set_Super_Capacitor_State(SUPERCAP_STATE_E SUPERCAP_STATE)
{
    if (SUPERCAP_STATE == SUPERCAP_ON)
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}
void UartTX_Super_Capacitor(int Power_Limitation, float Power)
{

    int IntIze_Power;
    uint8_t Buffer[11];
    IntIze_Power = (int)(Power * 10);
    Buffer[0] = '*';
    Buffer[1] = SuperCap_Power_TX_Typecode;
    Buffer[2] = (uint8_t)(Power_Limitation / 100);
    Power_Limitation = Power_Limitation - Buffer[2] * 100;
    Buffer[3] = (uint8_t)(Power_Limitation / 10);
    Buffer[4] = (uint8_t)(Power_Limitation % 10);
    Buffer[5] = (uint8_t)(IntIze_Power / 1000);
    IntIze_Power = IntIze_Power - Buffer[5] * 1000;
    Buffer[6] = (uint8_t)(IntIze_Power / 100);
    IntIze_Power = IntIze_Power - Buffer[6] * 100;
    Buffer[7] = (uint8_t)(IntIze_Power / 10);
    Buffer[8] = (uint8_t)(IntIze_Power % 10);
    Buffer[9] = 0;
    Buffer[10] = ';';
    uint8_t status;
    status = HAL_UART_Transmit(&huart1, Buffer, 11, 0xff);
}

void Supercap_Task(void)
{

    // Supercap_Keep_Alive();
    chassis.supercap.state=connection.connection_rx.supercap.flag;
    if (time.ms_count + time.s_count * 1000 - chassis.supercap.alive_ms - chassis.supercap.alive_s * 1000 > 1500)
    {
        chassis.supercap.online_state = SUPERCAP_OFFLINE;
    }
    else
    {
        chassis.supercap.online_state = SUPERCAP_ONLINE;
    }
    
    chassis.supercap.online_state = SUPERCAP_ONLINE;
    //Set_Super_Capacitor_State(chassis.supercap.state);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    Tx_Super_Capacitor();
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//     if (huart->Instance == USART1)
//     {
//         // 处理接收到的数据
//         if (supercap_rx_buffer[0] != '*' || supercap_rx_buffer[10] != ';')
//         {
//             // 数据格式错误
//             return;
//         }
//         else
//         {
//             switch (supercap_rx_buffer[1])
//             {
//             case Super_Cap_RX_Typecode:
//             {
//                 chassis.supercap.state = supercap_rx_buffer[2];
//                 chassis.supercap.supercap_per = supercap_rx_buffer[3];
//                 chassis.supercap.supercap_voltage = supercap_rx_buffer[4];
//             }
//             break;

//            case SuperCap_Status_RX_Typecode:
//            {
//                if (chassis.supercap.KeepAlive_SentData[0] == supercap_rx_buffer[2] &&
//                    chassis.supercap.KeepAlive_SentData[1] == supercap_rx_buffer[3] &&
//                    chassis.supercap.KeepAlive_SentData[2] == supercap_rx_buffer[4] &&
//                    chassis.supercap.KeepAlive_SentData[3] == supercap_rx_buffer[5])
//                {
//                    chassis.supercap.Keep_Alive_Time_Cnt = 0;
//                }
//            }
//            break;

//            default:
//                break;
//            }
//        }

//        HAL_UART_Receive_IT(&huart1, supercap_rx_buffer, 11);
//    }
//}