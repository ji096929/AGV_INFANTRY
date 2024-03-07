接收英雄战车云台的开发板的各种指令来控制英雄战车底盘实现各种动作（英雄底盘轮子部分的移动也可以直接接收遥控器控制）
（具体功能在【功能说明.txt】里）

# 开发者日志
- 2022.11.7 修改人：尹隽骞
    1. 增加了舵轮底盘的运动解算（暂未调用）。
- 2022.1.22 修改人：王晨阳
    1. 完成

# 硬件资源分配
开发板：C版
Device:STM32F407IGHx

 - GPIO:GPIOD Pin_0和Pin_1(CAN1)、GPIOB Pin_5和Pin_6(CAN2)、GPIOB Pin_7和GPIOA Pin_9（USART1)、GPIOC Pin_11和Pin_10(USART3)、GPIOG Pin_14和Pin_9(USART6)
 - CAN：CAN1、CAN2
 - 串口通信：USART1、USART3、USART6
 - 定时器：TIM3