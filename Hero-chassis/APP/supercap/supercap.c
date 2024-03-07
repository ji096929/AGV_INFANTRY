/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin, 
  *                     https://--
  *    
  * FileName   : supercap.c   
  * Version    : v1.0		
  * Author     : NEUQRM, DGYin, 		
  * Date       : 2023-05-29         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
  ******************************************************************************
 */
 
/*include----------------------------------------------------------------------*/
//#include "*.h"

/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/


/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/


/*Function prototype End*******************************************************/
/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      COPYRIGHT INFORMATION       ********************
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

#include "usart.h"
#include "supercap.h"
#include "referee.h"

/***********************************************************
*@Brief	供外部调用的变量
***********************************************************/
int Supercap_Connection_Status = Supercap_Disconnected;
int Supercap_UART_RX_KeepAlive_Flag = 0;
int Capacitor_State = 0; 					//电容剩余能量状态。0为低，1为高
int supercap_volt = 150; 					//超级电容电压
int Power_Mode = 0, Last_Power_Mode = 0;	//用于指示底盘加速模式
float supercap_per = 0; 					//超级电容电量百分比

/***********************************************************
*@Brief	仅供supercap.c文件中使用的变量
***********************************************************/
int Global_Time = 0;

/***********************************************************
*@Brief	调整supercap.c函数声明顺序
***********************************************************/
void Supercap_Trans_RefereeData(void);
void Supercap_Keep_Alive(void);
void PowerMode_Judgement(void);
void BetaBoard_Trans_PowerMode(void);

/***********************************************************
  * @breif         超级电容，根据不同的功率上限给电容上单片机发送数据
  * @param[in]     none
	* @param[out]    代表不同功率的字符
  * @retval        none
***********************************************************/

extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
extern void canTX_To_Beta_Power_Limit(int Power_Mode);
void supercap(int S_Cnt, int MS_Cnt)
{
	Global_Time = S_Cnt*1000 + MS_Cnt;	//计算全局时间
	if (Global_Time%200 == 0)
		Supercap_Keep_Alive();
	if (Global_Time%100 == 0)
	{
		Supercap_Trans_RefereeData();
		PowerMode_Judgement();
		BetaBoard_Trans_PowerMode();
	}
}
/***********************************************************
*@fuction	:Supercap_Keep_Alive
*@brief		:确认超电连接状态
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
extern void UART_TX_Supercap_Connection_Check(int Global_Time);
int Keep_Alive_Time_Cnt;
void Supercap_Keep_Alive(void)
{
	Keep_Alive_Time_Cnt++;	//计时
	UART_TX_Supercap_Connection_Check(Global_Time);	//发送查询请求
	//在bsp_uart.c的串口中断中，接收到查询反馈后，Keep_Alive_Time_Cnt将置零
	if (Keep_Alive_Time_Cnt > 10)//1.2s没有收到超电反馈后
		Supercap_Connection_Status = Supercap_Disconnected;
	else Supercap_Connection_Status = Supercap_Connected;
}

/***********************************************************
*@fuction	:Supercap_Trans_RefereeData
*@brief		:向超电发送裁判系统数据
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void Supercap_Trans_RefereeData(void)
{
	//向超电发送功率信息
	uint16_t UART_TX_Power_Max = 0, UART_TX_Buffer = 0;
	fp32 UART_TX_Power = 0;
    get_chassis_power_and_buffer_and_max(&UART_TX_Power, &UART_TX_Buffer, &UART_TX_Power_Max);  //获取裁判系统数据
    UartTX_Super_Capacitor(UART_TX_Power_Max, UART_TX_Power);	//向超电发送功率限制数据与底盘当前功耗
}

/***********************************************************
*@fuction	:PowerMode_Judgement
*@brief		:根据超电状态控制底盘功率模式
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void PowerMode_Judgement(void)
{
	//底盘功率模式判断，分超电是否连接两种情况
	if (Supercap_Connection_Status == Supercap_Connected)
	{
		if (supercap_volt > 200)
			Power_Mode = High_Voltage_Mode;
		if (supercap_volt <= 150)
			Power_Mode = Low_Voltage_Mode;
		if (Last_Power_Mode == Low_Voltage_Mode && supercap_volt < 180) Power_Mode = Low_Voltage_Mode; 
		else Power_Mode = High_Voltage_Mode;
		
	}
	else 
	{
		Power_Mode = Medium_Voltage_Mode;
		
	}
	//记录上次功率模式
    Last_Power_Mode = Power_Mode;
}

void BetaBoard_Trans_PowerMode(void)
{
	canTX_To_Beta_Power_Limit(Power_Mode);
}