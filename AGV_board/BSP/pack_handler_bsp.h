/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PACK_HANDLER_BSP_H
#define PACK_HANDLER_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>



typedef struct
{
	
	
} pack_t;

typedef struct
{
	uint8_t maximun_pack_num;
	pack_t *pack[maximun_pack_num];
	uint8_t queue_;
} pack_queue_t;



#ifdef __cplusplus
}
#endif
#endif