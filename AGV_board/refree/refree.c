#include "refree.h"
#include "usart.h"

unsigned char SaveBuffer[90];
float Last_chassisPower=0;
uint32_t SampleTick=0,SampleCount=0,Last_SampleTick=0,delt_Tick=0,Less60=0,Less100=0,From100_200=0,More200=0;
char TickCount=0;
uint16_t receivePower;
JudgeReceive_t JudgeReceive;
unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
void DMA1_Channel6_IRQHandler(void)
{
if (HAL_DMA_GetState(&hdma_usart2_rx) != HAL_DMA_STATE_READY)
{
  HAL_UART_DMAStop(&huart2);  
	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_TC6);
    JudgeBuffReceive(JudgeReceiveBuffer, 0);
	HAL_UART_Receive_DMA(&huart2,rx_buffer,90);
}
}

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen)
{
	uint16_t cmd_id;
	short PackPoint;
	memcpy(&SaveBuffer[JudgeBufBiggestSize],&rx_buffer[0],JudgeBufBiggestSize);		//把ReceiveBuffer[0]地址拷贝到SaveBuffer[24], 依次拷贝24个, 把这一次接收的存到数组后方
	for(PackPoint=0;PackPoint<JudgeBufBiggestSize;PackPoint++)		//先处理前半段数据(在上一周期已接收完成)
	{
		if(SaveBuffer[PackPoint]==0xA5) 
		{	
			if((Verify_CRC8_Check_Sum(&SaveBuffer[PackPoint],5)==1))		//frame_header 五位数据校验
			{
				cmd_id=(SaveBuffer[PackPoint+6])&0xff;
				cmd_id=(cmd_id<<8)|SaveBuffer[PackPoint+5];  
				DataLen=SaveBuffer[PackPoint+2]&0xff;
				DataLen=(DataLen<<8)|SaveBuffer[PackPoint+1];
				
				//机器人状态数据
				if((cmd_id==0x0201)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9))) 
				{
					memcpy(&JudgeReceive.robot_id,&SaveBuffer[PackPoint+7+0],1);
					memcpy(&JudgeReceive.RobotLevel,&SaveBuffer[PackPoint+7+1],1);
					memcpy(&JudgeReceive.remainHP,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&JudgeReceive.maxHP,&SaveBuffer[PackPoint+7+4],2);
					memcpy(&JudgeReceive.HeatCool42,&SaveBuffer[PackPoint+7+6],2);
					memcpy(&JudgeReceive.HeatMax42,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&JudgeReceive.MaxPower,&SaveBuffer[PackPoint+7+10],2);
					
				  
				}
					
				//实时功率、热量数据
				if((cmd_id==0x0202)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&JudgeReceive.realChassisOutV,&SaveBuffer[PackPoint+7+0],2);
					memcpy(&JudgeReceive.realChassisOutA,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&JudgeReceive.realChassispower,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&JudgeReceive.remainEnergy,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&JudgeReceive.shooterHeat42,&SaveBuffer[PackPoint+7+14],2);                              // 2个字节

					
					TickCount++;
					if(TickCount==5)
					{
					TickCount=0;
					}
					if(JudgeReceive.realChassispower!=Last_chassisPower)
					{
					

					}
					Last_chassisPower=JudgeReceive.realChassispower;
				}
				
				//实时射击信息
					if((cmd_id==0x0207)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&JudgeReceive.bulletFreq, &SaveBuffer[PackPoint+7+2],1);
					memcpy(&JudgeReceive.bulletSpeed,&SaveBuffer[PackPoint+7+3],4);
					JudgeReceive.ShootCpltFlag = 1;

				}
				
			}
		}

	memcpy(&SaveBuffer[0],&SaveBuffer[JudgeBufBiggestSize],JudgeBufBiggestSize);		//把SaveBuffer[24]地址拷贝到SaveBuffer[0], 依次拷贝24个，把之前存到后面的数据提到前面，准备处理
	}
}