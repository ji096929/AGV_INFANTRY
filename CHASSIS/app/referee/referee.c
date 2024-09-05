#include "referee.h"
#include "usart.h"
#include "chassis_task.h"
#include "fifo.h"
#include "algorithmOfCRC.h"

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
frame_header_struct_t referee_receive_header;
JudgeReceive_t JudgeReceive;

unpack_data_t referee_unpack_obj;

void ReFeree_Usart_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	__HAL_DMA_DISABLE(&hdma_usart6_rx);

	while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF2);

	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);

	hdma_usart6_rx.Instance->M0AR = (uint32_t)(usart6_buf);

	hdma_usart6_rx.Instance->M1AR = (uint32_t)(usart6_buf);

	__HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

	__HAL_DMA_ENABLE(&hdma_usart6_rx);

	__HAL_DMA_DISABLE(&hdma_usart6_tx);

	while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}

	hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void USART6_IRQHandler(void)
{
	static volatile uint8_t res;
	if (USART6->SR & UART_FLAG_IDLE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);

		static uint16_t this_time_rx_len = 0;

		if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(huart6.hdmarx);
			this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
			__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
			huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(huart6.hdmarx);
			fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);

			JudgeReceive.receive_flag = 1;
		}
		else
		{
			__HAL_DMA_DISABLE(huart6.hdmarx);
			this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
			__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
			huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(huart6.hdmarx);
			fifo_s_puts(&referee_fifo, (char *)usart6_buf[1], this_time_rx_len);
			JudgeReceive.receive_flag = 1;
		}
		
	}

HAL_UART_IRQHandler(&huart6);
}

void Referee_Init(void)
{
	ReFeree_Usart_Init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
	fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
}

void Judge_Buffer_Receive_Task(uint8_t *frame)
{
	uint16_t cmd_id = 0;

	uint8_t index = 0;

	memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

	index += sizeof(frame_header_struct_t);

	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

	switch (cmd_id)
	{

	case GAME_STATE_CMD_ID:
		memcpy(&JudgeReceive.game_type, frame + index, 1);

		break;
	case ROBOT_STATE_CMD_ID:
		memcpy(&JudgeReceive.robot_id, frame + index, 13);

		break;
	case POWER_HEAT_DATA_CMD_ID:
		memcpy(&JudgeReceive.realChassisOutV, frame + index, 12);

		break;
	case SHOOT_DATA_CMD_ID:
		memcpy(&JudgeReceive.bullet_type, frame + index, 7);

		break;


		
	default:
	{
		break;
	}
	}
}

void referee_unpack_fifo_data(void)
{
	uint8_t byte = 0;
	uint8_t sof = HEADER_SOF;
	unpack_data_t *p_obj = &referee_unpack_obj;

	while (fifo_s_used(&referee_fifo))
	{
		byte = fifo_s_get(&referee_fifo);

		switch (p_obj->unpack_step)
		{
		case STEP_HEADER_SOF:
		{
			if (byte == sof)
			{
				p_obj->unpack_step = STEP_LENGTH_LOW;
				p_obj->protocol_packet[p_obj->index++] = byte;
			}
			else
			{
				p_obj->index = 0;
			}
		}
		break;

		case STEP_LENGTH_LOW:
		{
			p_obj->data_len = byte;
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_LENGTH_HIGH;
		}
		break;

		case STEP_LENGTH_HIGH:
		{
			p_obj->data_len |= (byte << 8);
			p_obj->protocol_packet[p_obj->index++] = byte;

			if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
			{
				p_obj->unpack_step = STEP_FRAME_SEQ;
			}
			else
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
			}
		}
		break;
		case STEP_FRAME_SEQ:
		{
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_HEADER_CRC8;
		}
		break;

		case STEP_HEADER_CRC8:
		{
			p_obj->protocol_packet[p_obj->index++] = byte;

			if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
			{
				if (Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
				{
					p_obj->unpack_step = STEP_DATA_CRC16;
				}
				else
				{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}
			}
		}
		break;

		case STEP_DATA_CRC16:
		{
			if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
			{
				p_obj->protocol_packet[p_obj->index++] = byte;
			}
			if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;

				if (Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
				{
					Judge_Buffer_Receive_Task(p_obj->protocol_packet);
				}
			}
		}
		break;

		default:
		{
			p_obj->unpack_step = STEP_HEADER_SOF;
			p_obj->index = 0;
		}
		break;
		}
	}
}
