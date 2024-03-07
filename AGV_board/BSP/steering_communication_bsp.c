#include "steering_communication_bsp.h"





uint8_t steering_communication_transmit(steering_communication_ctx_t *ctx, steering_communication_pack_t *pack)
{
	// 拼接拓展帧
	uint32_t Extid;
	memcpy(&Extid,														&pack->steering_id,	sizeof(pack->steering_id));
	memcpy(&Extid + sizeof(pack->steering_id),							&pack->data2,		sizeof(pack->data2));
	memcpy(&Extid + sizeof(pack->steering_id) + sizeof(pack->data2),	&pack->cmd_id,		sizeof(pack->cmd_id));
	// 调用发送函数
	pack->treated_flag = ctx->tx_cmd(ctx->handle, Extid, pack->data1); // 如果发送成功，则声明数据包已经发送
	return pack->treated_flag;
}

steering_communication_pack_t steering_communication_receive_unpack(uint32_t extid, uint8_t *data1)
{
	steering_communication_pack_t pack;
	memcpy(&pack.data1,		data1, 8);
	memcpy(&pack.steering_id,	&extid, 4);
	return pack;
}
