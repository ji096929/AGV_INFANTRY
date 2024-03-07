/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_BSP_H
#define CAN_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105

typedef enum
{
	EXT_ID_MODE	=	0x00u,
	STD_ID_MODE	=	0x01u,
	EQUIPMENT_ID_IMPROVE	=0X02U,
	EQUIPMENT_ID_NORMAL		=0X04U,
	CMD_ID_IMPROVE				=	0X08U,
	CMD_ID_NORMAL					=	0X10U,
	DATA2_IMPROVE					=	0x20U,
	DATA2_NORMAL					=	0X40U,
}CAN_FILTER_IMPROVE_E;


#ifdef __cplusplus
}
#endif
#endif
