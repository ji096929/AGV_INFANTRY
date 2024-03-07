/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2022, Hebei, China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin, 
  *                     https://--
  *    
  * FileName   : lk_pitch_turn.c   
  * Version    : v1.0		
  * Author     : NEUQRM, DGYin, 
  * Date       : 2022-11-21         
  * Description: 封装了瓴控Pitch电机的基本操作    
  * Function List:  
  	1. ....
			<version>       : 		
			<modify staff>	:
			<data>          :
			<description>   : 
  	2. ...
  ******************************************************************************
 */
 
/*include----------------------------------------------------------------------*/
//#include "*.h"

#include "bsp_can.h"
#include "bsp_uart.h"


/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/
Briter_Encoder_t Briter_Encoder1,Briter_Encoder2,Briter_Encoder3,Briter_Encoder4;
/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/


/*Function prototype End*******************************************************/
/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      ESSESTIAL INFORMATION       ********************
 * ********************                                  ********************
 * **************************************************************************
 *                                                                          *
 *                                   _oo8oo_                                *
 *                                  o8888888o                               *
 *                                  88" . "88                               *
 *                                  (| -_- |)                               *
 *                                  0\  =  /0                               *
 *                                ___/'==='\___                             *
 *                              .' \\|     |// '.                           *
 *                             / \\|||  :  |||// \                          *
 *                            / _||||| -:- |||||_ \                         *
 *                           |   | \\\  -  /// |   |                        *
 *                           | \_|  ''\---/''  |_/ |                        *
 *                           \  .-\__  '-'  __/-.  /                        *
 *                         ___'. .'  /--.--\  '. .'___                      *
 *                      ."" '<  '.___\_<|>_/___.'  >' "".                   *
 *                     | | :  `- \`.:`\ _ /`:.`/ -`  : | |                  *
 *                     \  \ `-.   \_ __\ /__ _/   .-` /  /                  *
 *                 =====`-.____`.___ \_____/ ___.`____.-`=====              *
 *                                   `=---=`                                *
 * **************************************************************************
 * ********************                                  ********************
 * ********************      				 										 ********************
 * ********************         佛祖保佑 永远无BUG       ********************
 * ********************                                  ********************
 * **************************************************************************
 *         .............................................
 *                  佛祖镇楼                  BUG辟易
 *          佛曰:
 *                  写字楼里写字间，写字间里程序员；
 *                  程序人员写程序，又拿程序换酒钱。
 *                  酒醒只在网上坐，酒醉还来网下眠；
 *                  酒醉酒醒日复日，网上网下年复年。
 *                  但愿老死电脑间，不愿鞠躬老板前；
 *                  奔驰宝马贵者趣，公交自行程序员。
 *                  别人笑我忒疯癫，我笑自己命太贱；
 *                  不见满街漂亮妹，哪个归得程序员？
 */

/**
 **************************************************************
 *                                                            *
 *   .=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-.       *
 *    |                     ______                     |      *
 *    |                  .-"      "-.                  |      *
 *    |                 /            \                 |      *
 *    |     _          |              |          _     |      *
 *    |    ( \         |,  .-.  .-.  ,|         / )    |      *
 *    |     > "=._     | )(__/  \__)( |     _.=" <     |      *
 *    |    (_/"=._"=._ |/     /\     \| _.="_.="\_)    |      *
 *    |           "=._"(_     ^^     _)"_.="           |      *
 *    |               "=\__|IIIIII|__/="               |      *
 *    |              _.="| \IIIIII/ |"=._              |      *
 *    |    _     _.="_.="\          /"=._"=._     _    |      *
 *    |   ( \_.="_.="     `--------`     "=._"=._/ )   |      *
 *    |    > _.="                            "=._ <    |      *
 *    |   (_/                                    \_)   |      *
 *    |                                                |      *
 *    '-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-='      *
 *                                                            *
 *           LASCIATE OGNI SPERANZA, VOI CH'ENTRATE           *
 **************************************************************
 */
/*<!--
                       ::
                      :;J7, :,                        ::;7:
                      ,ivYi, ,                       ;LLLFS:
                      :iv7Yi                       :7ri;j5PL
                     ,:ivYLvr                    ,ivrrirrY2X,
                     :;r@Wwz.7r:                :ivu@kexianli.
                    :iL7::,:::iiirii:ii;::::,,irvF7rvvLujL7ur
                   ri::,:,::i:iiiiiii:i:irrv177JX7rYXqZEkvv17
                ;i:, , ::::iirrririi:i:::iiir2XXvii;L8OGJr71i
              :,, ,,:   ,::ir@mingyi.irii:i:::j1jri7ZBOS7ivv,
                 ,::,    ::rv77iiiriii:iii:i::,rvLq@huhao.Li
             ,,      ,, ,:ir7ir::,:::i;ir:::i:i::rSGGYri712:
           :::  ,v7r:: ::rrv77:, ,, ,:i7rrii:::::, ir7ri7Lri
          ,     2OBBOi,iiir;r::        ,irriiii::,, ,iv7Luur:
        ,,     i78MBBi,:,:::,:,  :7FSL: ,iriii:::i::,,:rLqXv::
        :      iuMMP: :,:::,:ii;2GY7OBB0viiii:i:iii:i:::iJqL;::
       ,     ::::i   ,,,,, ::LuBBu BBBBBErii:i:i:i:i:i:i:r77ii
      ,       :       , ,,:::rruBZ1MBBqi, :,,,:::,::::::iiriri:
     ,               ,,,,::::i:  @arqiao.       ,:,, ,:::ii;i7:
    :,       rjujLYLi   ,,:::::,:::::::::,,   ,:i,:,,,,,::i:iii
    ::      BBBBBBBBB0,    ,,::: , ,:::::: ,      ,,,, ,,:::::::
    i,  ,  ,8BMMBBBBBBi     ,,:,,     ,,, , ,   , , , :,::ii::i::
    :      iZMOMOMBBM2::::::::::,,,,     ,,,,,,:,,,::::i:irr:i:::,
    i   ,,:;u0MBMOG1L:::i::::::  ,,,::,   ,,, ::::::i:i:iirii:i:i:
    :    ,iuUuuXUkFu7i:iii:i:::, :,:,: ::::::::i:i:::::iirr7iiri::
    :     :rk@Yizero.i:::::, ,:ii:::::::i:::::i::,::::iirrriiiri::,
     :      5BMBBBBBBSr:,::rv2kuii:::iii::,:i:,, , ,,:,:i@petermu.,
          , :r50EZ8MBBBBGOBBBZP7::::i::,:::::,: :,:,::i;rrririiii::
              :jujYY7LS0ujJL7r::,::i::,::::::::::::::iirirrrrrrr:ii:
           ,:  :@kevensun.:,:,,,::::i:i:::::,,::::::iir;ii;7v77;ii;i,
           ,,,     ,,:,::::::i:iiiii:i::::,, ::::iiiir@xingjief.r;7:i,
        , , ,,,:,,::::::::iiiiiiiiii:,:,:::::::::iiir;ri7vL77rrirri::
         :,, , ::::::::i:::i:::i:i::,,,,,:,::i:i:::iir;@Secbone.ii:::

--*/

void Init_Encoder_Struct(void)
{
	Send_Data_Initialize();
 
	Briter_Encoder1.Request_Flag = 0;
	Briter_Encoder2.Request_Flag = 0;
	Briter_Encoder3.Request_Flag = 0;
	Briter_Encoder4.Request_Flag = 0;
	
}

void Briter_Encoder_Task(int S_Cnt, int MS_Cnt)
{
	int Global_Time;
	Global_Time = S_Cnt*1000 + MS_Cnt;
	if (Global_Time % 10 == 0)
		Get_Encoder_Position(1);
	if (Global_Time % 10 == 1)
		Get_Encoder_Position(2);
	if (Global_Time % 10 == 2)
		Get_Encoder_Position(3);
	if (Global_Time % 10 == 3)
		Get_Encoder_Position(4);
}
void Get_Encoder_Position(int ID) //如果ID为0，则查询四个编码器的数据。否则查询对应ID的编码器数据。结果存储在数据体中。
{
	Send_Data_Initialize();
    Briter_Encoder1.Length = 0x04; Briter_Encoder1.Order_Code = 0x01;
    Briter_Encoder2.Length = 0x04; Briter_Encoder2.Order_Code = 0x01;
    Briter_Encoder3.Length = 0x04; Briter_Encoder3.Order_Code = 0x01;
    Briter_Encoder4.Length = 0x04; Briter_Encoder4.Order_Code = 0x01;
	Briter_Send_Data(ID);
}

void Set_Encoder_Callback_Mode(int ID, int Mode)
{
	Send_Data_Initialize();
	Briter_Encoder1.Length = 0x04; Briter_Encoder1.Order_Code = 0x04;
    Briter_Encoder2.Length = 0x04; Briter_Encoder2.Order_Code = 0x04;
    Briter_Encoder3.Length = 0x04; Briter_Encoder3.Order_Code = 0x04;
    Briter_Encoder4.Length = 0x04; Briter_Encoder4.Order_Code = 0x04;
	if (Mode == Briter_Request_Mode) //查询模式 
	{
		Briter_Encoder1.Send_Data[3]=0x00; Briter_Encoder2.Send_Data[3]=0x00; Briter_Encoder3.Send_Data[3]=0x00; Briter_Encoder4.Send_Data[3]=0x00;
	}
	else if (Mode == Briter_Auto_Callback_Mode) //自动回发模式
	{
		Briter_Encoder1.Send_Data[3]=0xAA; Briter_Encoder2.Send_Data[3]=0xAA; Briter_Encoder3.Send_Data[3]=0xAA; Briter_Encoder4.Send_Data[3]=0xAA;
	}
    
	Briter_Send_Data(ID); //调用can发送函数，将数据发给编码器
}

void Set_Encoder_Callback_Period(int ID, int Period)
{
	Send_Data_Initialize();
	Briter_Encoder1.Length = 0x05; Briter_Encoder1.Order_Code = 0x05;
    Briter_Encoder2.Length = 0x05; Briter_Encoder2.Order_Code = 0x05;
    Briter_Encoder3.Length = 0x05; Briter_Encoder3.Order_Code = 0x05;
    Briter_Encoder4.Length = 0x05; Briter_Encoder4.Order_Code = 0x05;
	
	Briter_Encoder1.Send_Data[3]=Period&0xFF; Briter_Encoder1.Send_Data[4]=Period>>8;
	Briter_Encoder2.Send_Data[3]=Period&0xFF; Briter_Encoder2.Send_Data[4]=Period>>8;
	Briter_Encoder3.Send_Data[3]=Period&0xFF; Briter_Encoder3.Send_Data[4]=Period>>8;
	Briter_Encoder4.Send_Data[3]=Period&0xFF; Briter_Encoder4.Send_Data[4]=Period>>8;
    
	Briter_Send_Data(ID); //调用can发送函数，将数据发给编码器
}

void Briter_Send_Data(int ID)
{
	 switch(ID)
    {       
        case 1:   
			canTX_Briter_Encoder(Briter_Encoder1,&hcan2);
		break;
        case 2:    
			canTX_Briter_Encoder(Briter_Encoder2,&hcan1);
			//UartTX_To_BetaBoard_Briter_Encoder(Briter_Encoder2,Briter_Encoder2_TypeCode);
		break;
        case 3:
			canTX_Briter_Encoder(Briter_Encoder3,&hcan2);
        break;
        case 4:  
			canTX_Briter_Encoder(Briter_Encoder4,&hcan1);			
			//UartTX_To_BetaBoard_Briter_Encoder(Briter_Encoder4,Briter_Encoder4_TypeCode);
        break;
        
        case 0://
			Briter_Send_Data(1);
            Briter_Send_Data(2);
            Briter_Send_Data(3);
            Briter_Send_Data(4);
        break;
	}
}

void Send_Data_Initialize(void)
{
	for (int i=0; i<8; i++)
	{
		Briter_Encoder1.Send_Data[i] = 0;
		Briter_Encoder2.Send_Data[i] = 0;
		Briter_Encoder3.Send_Data[i] = 0;
		Briter_Encoder4.Send_Data[i] = 0;
	}
}


int All_Initial_Position_Recorded(void)
{
	if ((Briter_Encoder1.Request_Flag == 1) && (Briter_Encoder2.Request_Flag == 1) && (Briter_Encoder3.Request_Flag == 1) && (Briter_Encoder4.Request_Flag == 1))
		return 1;
	else
		return 0;
}