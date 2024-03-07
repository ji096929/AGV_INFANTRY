#include "referee_UI.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "referee.h"
#include "bsp_referee.h"
#include "supercap.h"
#include "relay_task.h"

extern int time2;
#define Max(a,b) ((a) > (b) ? (a) : (b))
#define Robot_ID_Current Robot_ID_Red_Infantry3
/* 绘制UI专用结构体 */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_String_t UI_String1;
UI_Delete_t UI_Delete;
uint8_t seq = 0;
//-------------------------
uint8_t UI_AutoAim_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Gimbal_Pitch = 1.000f; //云台Pitch轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
uint8_t UI_Capacitance  = 50;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启
char vision_mode_[1] = "v";
char chassis_mode_[12] = "chassis_mode";

/* 中央标尺高度变量 */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;

//只在UI中用到的变量
int Shoot_Num = 0, Fric_State = 0;
int Pitch_Tempreture;
extern int supercap_volt;
uint8_t autoaim_mode;//2:normal,3:small energy,4:big energy
uint8_t autoaim_armor;//0x10:auto,0x20:big,0x30:small
uint8_t if_predict;

extern void get_shoot_data(uint8_t *bullet_freq,	float *bullet_speed, uint8_t *bullet_speedlimit);
//----------------------------

static void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    unsigned char i = i;

    uint8_t tx_buff[MAX_SIZE];

    uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度

    memset(tx_buff, 0, frame_length); //存储数据的数组清零

    /*****帧头打包*****/
    tx_buff[0] = sof;//数据帧起始字节
    memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len)); //数据帧中data的长度
    tx_buff[3] = seq;//包序号
    append_CRC8_check_sum(tx_buff, frameheader_len); //帧头校验CRC8

    /*****命令码打包*****/
    memcpy(&tx_buff[frameheader_len], (uint8_t *)&cmd_id, cmd_len);

    /*****数据打包*****/
    memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);
    append_CRC16_check_sum(tx_buff, frame_length); //一帧数据校验CRC16

    if (seq == 0xff) seq = 0;
    else seq++;

    /*****数据上传*****/
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
    HAL_UART_Transmit(&huart6, tx_buff, frame_length, 100);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET); //等待之前的字符发送完成
}

static void get_UI_id(uint16_t *sender_ID, uint16_t *receiver_ID)
{
    switch(get_robot_id())
    {
    case UI_Data_RobotID_RHero:
    {
        *sender_ID = UI_Data_RobotID_RHero;
        *receiver_ID = UI_Data_CilentID_RHero;
        break;
    }
    case UI_Data_RobotID_REngineer:
    {
        *sender_ID = UI_Data_RobotID_REngineer;
        *receiver_ID = UI_Data_CilentID_REngineer;
        break;
    }
    case UI_Data_RobotID_RStandard1:
    {
        *sender_ID = UI_Data_RobotID_RStandard1;
        *receiver_ID = UI_Data_CilentID_RStandard1;
        break;
    }
    case UI_Data_RobotID_RStandard2:
    {
        *sender_ID = UI_Data_RobotID_RStandard2;
        *receiver_ID = UI_Data_CilentID_RStandard2;
        break;
    }
    case UI_Data_RobotID_RStandard3:
    {
        *sender_ID = UI_Data_RobotID_RStandard3;
        *receiver_ID = UI_Data_CilentID_RStandard3;
        break;
    }
    case UI_Data_RobotID_RAerial:
    {
        *sender_ID = UI_Data_RobotID_RAerial;
        *receiver_ID = UI_Data_CilentID_RAerial;
        break;
    }
    case UI_Data_RobotID_BHero:
    {
        *sender_ID = UI_Data_RobotID_BHero;
        *receiver_ID = UI_Data_CilentID_BHero;
        break;
    }
    case UI_Data_RobotID_BEngineer:
    {
        *sender_ID = UI_Data_RobotID_BEngineer;
        *receiver_ID = UI_Data_CilentID_BEngineer;
        break;
    }
    case UI_Data_RobotID_BStandard1:
    {
        *sender_ID = UI_Data_RobotID_BStandard1;
        *receiver_ID = UI_Data_CilentID_BStandard1;
        break;
    }
    case UI_Data_RobotID_BStandard2:
    {
        *sender_ID = UI_Data_RobotID_BStandard2;
        *receiver_ID = UI_Data_CilentID_BStandard2;
        break;
    }
    case UI_Data_RobotID_BStandard3:
    {
        *sender_ID = UI_Data_RobotID_BStandard3;
        *receiver_ID = UI_Data_CilentID_BStandard3;
        break;
    }
    case UI_Data_RobotID_BAerial:
    {
        *sender_ID = UI_Data_RobotID_BAerial;
        *receiver_ID = UI_Data_CilentID_BAerial;
        break;
    }
    }
}

uint16_t Sender_ID, Receiver_ID;
/************************************************绘制直线*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/

void UI_Draw_Line(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
                  char                   GraphName[3], //图形名 作为客户端的索引
                  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                  uint8_t                Layer,        //UI图形图层 [0,9]
                  uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
                  uint16_t               Width,        //线宽
                  uint16_t               StartX,       //起始坐标X
                  uint16_t               StartY,       //起始坐标Y
                  uint16_t               EndX,         //截止坐标X
                  uint16_t               EndY)         //截止坐标Y
{
    Graph->graphic_name[0] = GraphName[0];
    Graph->graphic_name[1] = GraphName[1];
    Graph->graphic_name[2] = GraphName[2];
    Graph->operate_tpye    = GraphOperate;
    Graph->graphic_tpye    = UI_Graph_Line;
    Graph->layer           = Layer;
    Graph->color           = Color;
    Graph->width           = Width;
    Graph->start_x         = StartX;
    Graph->start_y         = StartY;
    Graph->end_x           = EndX;
    Graph->end_y           = EndY;


}

void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
                       char                   GraphName[3], //图形名 作为客户端的索引
                       uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                       uint8_t                Layer,        //UI图形图层 [0,9]
                       uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
                       uint16_t               Width,        //线宽
                       uint16_t               StartX,       //起始坐标X
                       uint16_t               StartY,       //起始坐标Y
                       uint16_t               EndX,         //截止坐标X
                       uint16_t               EndY)         //截止坐标Y
{
    Graph->graphic_name[0] = GraphName[0];
    Graph->graphic_name[1] = GraphName[1];
    Graph->graphic_name[2] = GraphName[2];
    Graph->operate_tpye    = GraphOperate;
    Graph->graphic_tpye    = UI_Graph_Rectangle;
    Graph->layer           = Layer;
    Graph->color           = Color;
    Graph->width           = Width;
    Graph->start_x         = StartX;
    Graph->start_y         = StartY;
    Graph->end_x           = EndX;
    Graph->end_y           = EndY;
}

void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
                    char                   GraphName[3], //图形名 作为客户端的索引
                    uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                    uint8_t                Layer,        //UI图形图层 [0,9]
                    uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
                    uint16_t               Width,        //线宽
                    uint16_t               CenterX,      //圆心坐标X
                    uint16_t               CenterY,      //圆心坐标Y
                    uint16_t               Radius)       //半径
{
    Graph->graphic_name[0] = GraphName[0];
    Graph->graphic_name[1] = GraphName[1];
    Graph->graphic_name[2] = GraphName[2];
    Graph->operate_tpye    = GraphOperate;
    Graph->graphic_tpye    = UI_Graph_Circle;
    Graph->layer           = Layer;
    Graph->color           = Color;
    Graph->width           = Width;
    Graph->start_x         = CenterX;
    Graph->start_y         = CenterY;
    Graph->radius          = Radius;

}

void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
                     char                   GraphName[3], //图形名 作为客户端的索引
                     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                     uint8_t                Layer,        //UI图形图层 [0,9]
                     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
                     uint16_t               Width,        //线宽
                     uint16_t               CenterX,      //圆心坐标X
                     uint16_t               CenterY,      //圆心坐标Y
                     uint16_t               XHalfAxis,    //X半轴长
                     uint16_t               YHalfAxis)    //Y半轴长
{
    Graph->graphic_name[0] = GraphName[0];
    Graph->graphic_name[1] = GraphName[1];
    Graph->graphic_name[2] = GraphName[2];
    Graph->operate_tpye    = GraphOperate;
    Graph->graphic_tpye    = UI_Graph_Ellipse;
    Graph->layer           = Layer;
    Graph->color           = Color;
    Graph->width           = Width;
    Graph->start_x         = CenterX;
    Graph->start_y         = CenterY;
    Graph->end_x           = XHalfAxis;
    Graph->end_y           = YHalfAxis;
}

void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
                 char                   GraphName[3], //图形名 作为客户端的索引
                 uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                 uint8_t                Layer,        //UI图形图层 [0,9]
                 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
                 uint16_t               StartAngle,   //起始角度 [0,360]
                 uint16_t               EndAngle,     //截止角度 [0,360]
                 uint16_t               Width,        //线宽
                 uint16_t               CenterX,      //圆心坐标X
                 uint16_t               CenterY,      //圆心坐标Y
                 uint16_t               XHalfAxis,    //X半轴长
                 uint16_t               YHalfAxis)    //Y半轴长
{
    Graph->graphic_name[0] = GraphName[0];
    Graph->graphic_name[1] = GraphName[1];
    Graph->graphic_name[2] = GraphName[2];
    Graph->operate_tpye    = GraphOperate;
    Graph->graphic_tpye    = UI_Graph_Arc;
    Graph->layer           = Layer;
    Graph->color           = Color;
    Graph->start_angle     = StartAngle;
    Graph->end_angle       = EndAngle;
    Graph->width           = Width;
    Graph->start_x         = CenterX;
    Graph->start_y         = CenterY;
    Graph->end_x           = XHalfAxis;
    Graph->end_y           = YHalfAxis;
}

void UI_Draw_Float(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
                   char                   GraphName[3], //图形名 作为客户端的索引
                   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                   uint8_t                Layer,        //UI图形图层 [0,9]
                   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
                   uint16_t               NumberSize,   //字体大小
                   uint16_t               Significant,  //有效位数
                   uint16_t               Width,        //线宽
                   uint16_t               StartX,       //起始坐标X
                   uint16_t               StartY,       //起始坐标Y
                   float                  FloatData)    //数字内容
{
    Graph->graphic_name[0] = GraphName[0];
    Graph->graphic_name[1] = GraphName[1];
    Graph->graphic_name[2] = GraphName[2];
    Graph->operate_tpye    = GraphOperate;
    Graph->graphic_tpye    = UI_Graph_Float;
    Graph->layer           = Layer;
    Graph->color           = Color;
    Graph->start_angle     = NumberSize;
    Graph->end_angle       = Significant;
    Graph->width           = Width;
    Graph->start_x         = StartX;
    Graph->start_y         = StartY;
    int32_t IntData = FloatData * 1000;
    Graph->radius          = (IntData & 0x000003ff) >>  0;
    Graph->end_x           = (IntData & 0x001ffc00) >> 10;
    Graph->end_y           = (IntData & 0xffe00000) >> 21;

}




void UI_Draw_String(string_data_struct_t *String,        //UI图形数据结构体指针
                    char                  StringName[3], //图形名 作为客户端的索引
                    uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
                    uint8_t               Layer,         //UI图形图层 [0,9]
                    uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
                    uint16_t              CharSize,      //字体大小
                    uint16_t              StringLength,  //字符串长度
                    uint16_t              Width,         //线宽
                    uint16_t              StartX,        //起始坐标X
                    uint16_t              StartY,        //起始坐标Y
                    char                 *StringData)    //字符串内容
{
    String->string_name[0] = StringName[0];
    String->string_name[1] = StringName[1];
    String->string_name[2] = StringName[2];
    String->operate_tpye   = StringOperate;
    String->graphic_tpye   = UI_Graph_String;
    String->layer          = Layer;
    String->color          = Color;
    String->start_angle    = CharSize;
    String->end_angle      = StringLength;
    String->width          = Width;
    String->start_x        = StartX;
    String->start_y        = StartY;
    for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}






void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* 与Counter相一致的UI_Graphx结构体头指针 */, uint8_t RobotID)
{
    UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //假设只发一个基本图形

    /* 填充 frame_header */
    Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
    if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
    else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
    else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
    else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
    Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
    Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);

    /* 填充 cmd_id */
    Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

    /* 填充 student_interactive_header */
    if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
    else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
    else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
    else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
    Graph->Interactive_Header.sender_ID   = RobotID ;      //当前机器人ID
    Graph->Interactive_Header.receiver_ID = RobotID + 256; //对应操作手ID

    /* 填充 frame_tail 即CRC16 */
    if(Counter == 1)
    {
        UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
        Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
    }
    else if(Counter == 2)
    {
        UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
        Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
    }
    else if(Counter == 5)
    {
        UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
        Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
    }
    else if(Counter == 7)
    {
        UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
        Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
    }

    /* 使用串口PushUp到裁判系统 */
    if(Counter == 1) HAL_UART_Transmit(&huart6, (uint8_t *)Graph, sizeof(UI_Graph1_t), 0xff);
    else if(Counter == 2) HAL_UART_Transmit(&huart6, (uint8_t *)Graph, sizeof(UI_Graph2_t), 0xff);
    else if(Counter == 5) HAL_UART_Transmit(&huart6, (uint8_t *)Graph, sizeof(UI_Graph5_t), 0xff);
    else if(Counter == 7) HAL_UART_Transmit(&huart6, (uint8_t *)Graph, sizeof(UI_Graph7_t), 0xff);
}

void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
    /* 填充 frame_header */
    String->Referee_Transmit_Header.SOF  = HEADER_SOF;
    String->Referee_Transmit_Header.data_length = 6 + 45;
    String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
    String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);

    /* 填充 cmd_id */
    String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

    /* 填充 student_interactive_header */
    String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
    String->Interactive_Header.sender_ID   = RobotID ;      //当前机器人ID
    String->Interactive_Header.receiver_ID = RobotID + 256; //对应操作手ID

    /* 填充 frame_tail 即CRC16 */
    String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);

    /* 使用串口PushUp到裁判系统 */
    HAL_UART_Transmit(&huart6, (uint8_t *)String, sizeof(UI_String_t), 0xff);
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
    /* 填充 frame_header */
    Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
    Delete->Referee_Transmit_Header.data_length = 6 + 2;
    Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
    Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);

    /* 填充 cmd_id */
    Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

    /* 填充 student_interactive_header */
    Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
    Delete->Interactive_Header.sender_ID   = RobotID ;      //当前机器人ID
    Delete->Interactive_Header.receiver_ID = RobotID + 256; //对应操作手ID

    /* 填充 frame_tail 即CRC16 */
    Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);

    /* 使用串口PushUp到裁判系统 */
    HAL_UART_Transmit(&huart6, (uint8_t *)Delete, sizeof(UI_Delete_t), 0xff);
}
uint16_t UI_PushUp_Counter = 0, UI_Init_Counter = 0;

int Referee_UI_Init_Flag = 0;
void Referee_UI_Init(void)
{
    UI_Init_Counter++; //计时用
    if (UI_Init_Counter % 1800 == 0) //定时重启
        Referee_UI_Init_Flag = 0;

    //绘制不变的线
    if(UI_Init_Counter % 111 == 0)//定时执行
    {
        switch(Referee_UI_Init_Flag)//初始化UI任务列表
        {
        //静态UI预绘制1
        case 0:
            //中央标尺绘制1
            UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,	0,	0,		0,	0); //第一行左横线
            UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  0,	0,		0,	0); //第一行十字横
            UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 	0,	0,		0,	0); //第一行右横线
            UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  0,	0,		0,	0); //第一行十字竖
            UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  0,	0,		0,	0); //第二行左横线
            UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  0,	0,		0,	0); //第二行中心点
            UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Change, 0, UI_Color_Green, 3, SXC, 335, 1920, 190); //第二行右横线
            UI_PushUp_Graphs(7, &UI_Graph7, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        //静态UI预绘制2
        case 1:
            //中央标尺绘制2
			float Bullet_Speed; uint8_t Useless_Var;
			get_shoot_data(&Useless_Var, &Bullet_Speed ,&Useless_Var);
            UI_Draw_Line(&UI_Graph7.Graphic[0], "008",	UI_Graph_Add,	0,	UI_Color_Yellow, 1, 0		, 0		, 0		, 0); //第三行左横线
            UI_Draw_Line(&UI_Graph7.Graphic[1], "009",	UI_Graph_Add,	0,	UI_Color_Yellow, 5, 0		, 0		, 0		, 0); //第三行中心点
            UI_Draw_Line(&UI_Graph7.Graphic[2], "010",	UI_Graph_Add,	0,	UI_Color_Yellow, 1, 0		, 0		, 0		, 0); //第三行右横线
            UI_Draw_Float(&UI_Graph7.Graphic[4], "210", UI_Graph_Add, 1, UI_Color_Yellow, 18, 1, 3, Right_Num_List_X, 500, Bullet_Speed);
            UI_Draw_Arc	(&UI_Graph7.Graphic[0],	"110",	UI_Graph_Add, 2, UI_Color_Green, 350, 10,	10, 1920/2,	1080/2,	500, 300);//底盘前部色环
            UI_Draw_Line(&UI_Graph7.Graphic[5], "013",	UI_Graph_Add,	0,	UI_Color_Yellow, 2, SXC - 10	, 309	, SXC + 10	, 309); //第四行右横线
            UI_Draw_Line(&UI_Graph7.Graphic[6], "014",	UI_Graph_Add,	0,	UI_Color_Yellow, 1, SXC		, 250	, SXC	, 700); //中心竖线
            UI_PushUp_Graphs(7, &UI_Graph7, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        //静态UI预绘制3, 动态UI预绘制1
        case 2:
            //小陀螺预警线
            UI_Draw_Line(&UI_Graph7.Graphic[0],	"101",	UI_Graph_Add,		1,	UI_Color_Yellow,	2,	630,	30,		780,	100);
            UI_Draw_Line(&UI_Graph7.Graphic[1],	"102",	UI_Graph_Add,		1,	UI_Color_Yellow,	2,	780,	100,	930,	100);
            UI_Draw_Line(&UI_Graph7.Graphic[2],	"103",	UI_Graph_Add,		1,	UI_Color_Yellow,	2,  990,	100,	1140,	100);
            UI_Draw_Line(&UI_Graph7.Graphic[3],	"104",	UI_Graph_Add,		1,	UI_Color_Yellow,	2,	1140,	100,	1290,	30);
            //UI_Draw_Line(&UI_Graph7.Graphic[4],	"105",	UI_Graph_Add,		1,	UI_Color_Yellow,	5,	959,	100,	960,	100);
            //模式色环的绘制
			UI_Draw_Arc	(&UI_Graph7.Graphic[4],	"109",	UI_Graph_Add,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	520,	15,	15);//跟随模式色环
            UI_Draw_Arc	(&UI_Graph7.Graphic[5],	"106",	UI_Graph_Add,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	660,	15,	15);//吊射模式色环
            UI_Draw_Arc	(&UI_Graph7.Graphic[6],	"107",	UI_Graph_Add,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	590,	15,	15);//陀螺模式色环
            UI_PushUp_Graphs(7, &UI_Graph7, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 3:
			UI_Draw_Float(&UI_Graph5.Graphic[4], "205", UI_Graph_Add, 1, UI_Color_White, 18, 0, 3, Right_Num_List_X, 692, supercap_volt);
            UI_Draw_Float(&UI_Graph5.Graphic[3], "201", UI_Graph_Add, 2, UI_Color_White, 18, 3, 3, Right_Num_List_X, 632, 0.00f);
            UI_Draw_Float(&UI_Graph5.Graphic[2], "204", UI_Graph_Add, 1, UI_Color_White, 18, 0, 3, Right_Num_List_X, 572, Pitch_Tempreture);
            UI_Draw_Arc	(&UI_Graph5.Graphic[1],	"108",	UI_Graph_Add,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	730,	15,	15);//摩擦轮色环
            UI_Draw_Float(&UI_Graph5.Graphic[0], "203", UI_Graph_Add, 1, UI_Color_White, 18, 0, 3, 1500, 632, Shoot_Num);
            UI_PushUp_Graphs(5, &UI_Graph5, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 4:
            UI_Draw_String(&UI_String1.String, "305", UI_Graph_Add, 2, UI_Color_Pink, 18, 8, 3,  Left_Title_List_X, 670, "Prcs");//吊射模式指示，粉色为关，绿色为开
            UI_PushUp_String(&UI_String1, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 5:
            UI_Draw_String(&UI_String1.String, "306", UI_Graph_Add, 2, UI_Color_Pink, 18, 8, 3,  Left_Title_List_X, 740, "Fric");//摩擦轮状态指示，粉色为关，绿色为开
            UI_PushUp_String(&UI_String1, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 6:
            UI_Draw_String(&UI_String1.String, "307", UI_Graph_Add, 2, UI_Color_White, 18, 8, 3,  Right_Title_List_X, 572, "PMCD:");//Pitch电机温度指示
            UI_PushUp_String(&UI_String1, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 7:
            UI_Draw_String(&UI_String1.String, "308", UI_Graph_Add, 2, UI_Color_White, 18, 8, 3,  Right_Title_List_X, 632, "LCC:");//左键次数指示
            UI_PushUp_String(&UI_String1, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 8:
            UI_Draw_String(&UI_String.String, "304", UI_Graph_Add, 2, UI_Color_Pink, 18, 8, 3,  Left_Title_List_X, 600, "Spin");//陀螺模式指示，粉色为关，绿色为开
            UI_PushUp_String(&UI_String, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
        case 9:
            UI_Draw_String(&UI_String1.String, "309", UI_Graph_Add, 2, UI_Color_White, 18, 8, 3,  Right_Title_List_X, 692, "SCV:");//超电电压指示
            UI_PushUp_String(&UI_String1, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
		case 10:
            UI_Draw_String(&UI_String.String, "310", UI_Graph_Add, 2, UI_Color_Pink, 18, 8, 3,  Left_Title_List_X, 530, "FOLW");//陀螺模式指示，粉色为关，绿色为开
            UI_PushUp_String(&UI_String, get_robot_id());
            Referee_UI_Init_Flag++;
            break;
			
        }
    }
}
void referee_usart_task(void const *argument)
{
    /* 动态UI控制变量 */
    extern int Chassis_Follow_Switch;
    /* 裁判系统初始化 */
    Referee_UI_Init();
    /* 解析裁判系统数据 */
    //vTaskDelay(10);
    //Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
    /* UI更新 */
    UI_PushUp_Counter++; //计时用
    /**************************************************/
    /*      1、不得占用取余为1,21的时间用于发送       */
    /**************************************************/
    if(UI_PushUp_Counter % 21 == 0) //静态UI 模式色环
    {
		extern float Relative_Angle_For_UI;
		Relative_Angle_For_UI = 360 - Relative_Angle_For_UI;
		float Head_Start_Angle, Head_End_Angle;
		Head_Start_Angle = Relative_Angle_For_UI - 17;
		if (Head_Start_Angle<0) Head_Start_Angle = Head_Start_Angle+360; if (Head_Start_Angle>360) Head_Start_Angle = Head_Start_Angle-360;
		Head_End_Angle = Relative_Angle_For_UI + 17;
		if (Head_End_Angle>360) Head_End_Angle = Head_End_Angle-360; if (Head_End_Angle<0) Head_End_Angle = Head_End_Angle+360;
		UI_Draw_Arc	(&UI_Graph7.Graphic[5],	"110",	UI_Graph_Change, 2, UI_Color_White, Head_Start_Angle, Head_End_Angle,	13, 1920/2,	1080/2,	387, 387);//底盘前部色环
		
		if (Chassis_Follow_Switch == 1)
			UI_Draw_Arc	(&UI_Graph7.Graphic[0],	"109",	UI_Graph_Change,	2,	UI_Color_Green,		0,	360,	10,		Left_Ring_List_X,	520,	15,	15);//跟随模式色环
		else	UI_Draw_Arc	(&UI_Graph7.Graphic[0],	"109",	UI_Graph_Change,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	520,	15,	15);//跟随模式色环
        UI_Draw_Float(&UI_Graph7.Graphic[1], "203", UI_Graph_Change, 1, UI_Color_White, 18, 0, 3, 1700, 632, Shoot_Num);
        if (Fric_State == 1)
            UI_Draw_Arc	(&UI_Graph7.Graphic[2],	"108",	UI_Graph_Change,	2,	UI_Color_Green,		0,	360,	10,		Left_Ring_List_X,	730,	15,	15);//摩擦轮色环
        else 	UI_Draw_Arc	(&UI_Graph7.Graphic[2],	"108",	UI_Graph_Change,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	730,	15,	15);//摩擦轮色环
        if (chassis_control_order.Precision_Mode == 0)
            UI_Draw_Arc	(&UI_Graph7.Graphic[3],	"106",	UI_Graph_Change,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	660,	15,	15);//吊射模式色环
        else 	UI_Draw_Arc	(&UI_Graph7.Graphic[3],	"106",	UI_Graph_Change,	2,	UI_Color_Green,		0,	360,	10,		Left_Ring_List_X,	660,	15,	15);//吊射模式色环
        if (chassis_control_order.chassis_mode == CHASSIS_SPIN)
            UI_Draw_Arc	(&UI_Graph7.Graphic[4],	"107",	UI_Graph_Change,	2,	UI_Color_Green,		0,	360,	10,		Left_Ring_List_X,	590,	15,	15);//陀螺模式色环
        else 	UI_Draw_Arc	(&UI_Graph7.Graphic[4],	"107",	UI_Graph_Change,	2,	UI_Color_Pink,		0,	360,	5,		Left_Ring_List_X,	590,	15,	15);//陀螺模式色环
        
		float Bullet_Speed; uint8_t Useless_Var;
		get_shoot_data(&Useless_Var, &Bullet_Speed ,&Useless_Var);
		UI_Draw_Float(&UI_Graph7.Graphic[4], "210", UI_Graph_Change, 1, UI_Color_Yellow, 18, 1, 3, Right_Num_List_X, 500, Bullet_Speed);
            
		UI_PushUp_Graphs(7, &UI_Graph7, get_robot_id());
    }
    if(UI_PushUp_Counter % 131 == 0) //动态UI预绘制 图形
    {
        UI_Draw_Float(&UI_Graph2.Graphic[0], "204", UI_Graph_Change, 1, UI_Color_White, 18, 0, 3, 1700, 572, Pitch_Tempreture);
        UI_Draw_Float(&UI_Graph2.Graphic[1],	"202",	UI_Graph_Add,	2,	UI_Color_Yellow,	20,	1829,	330,	1870,	832,	334.00f);     //电容容量
        //	UI_Draw_Arc(&UI_Graph2.Graphic[1],"204",UI_Graph_Change,2,UI_Color_Pink,0,360,2,10,10,5,5);
        UI_PushUp_Graphs(2, &UI_Graph2, get_robot_id());
    }

    if(UI_PushUp_Counter % 241 == 0) //动态UI绘制 小陀螺模式
    {
        UI_Draw_String(&UI_String.String, "304", UI_Graph_Change, 2, UI_Color_White, 18, 8, 3,  Left_Title_List_X, 600, "Spin");//陀螺模式指示，粉色为关，绿色为开
        UI_PushUp_String(&UI_String, get_robot_id());
    }
    if(UI_PushUp_Counter % 251 == 0) //动态UI绘制 吊射模式
    {
        UI_Draw_String(&UI_String1.String, "305", UI_Graph_Change, 2, UI_Color_White, 18, 8, 3,  Left_Title_List_X, 670, "Prcs");//吊射模式指示，粉色为关，绿色为开
        UI_PushUp_String(&UI_String1, get_robot_id());
    }
    if(UI_PushUp_Counter % 261 == 0) //动态UI绘制 摩擦轮状态
    {
        UI_Draw_String(&UI_String.String, "306", UI_Graph_Change, 2, UI_Color_White, 18, 8, 3,  Left_Title_List_X, 740, "Fric");//摩擦轮指示，粉色为关，绿色为开
		UI_PushUp_String(&UI_String, get_robot_id());
	}
	if(UI_PushUp_Counter % 271 == 0) //动态UI绘制 继电器状态
    {
        if (Relay_Set_State == Direct_Power_Supply)
		UI_Draw_String(&UI_String1.String, "309", UI_Graph_Change, 2, UI_Color_Pink, 22, 8, 5,  Right_Title_List_X, 692, "DIRC");//摩擦轮指示，粉色为关，绿色为开
        else UI_Draw_String(&UI_String1.String, "309", UI_Graph_Change, 2, UI_Color_White, 18, 8, 3,  Right_Title_List_X, 692, "SCV:");
		UI_PushUp_String(&UI_String1, get_robot_id());
    }
	if(UI_PushUp_Counter % 281 == 0) 
	{
		UI_Draw_String(&UI_String.String, "310", UI_Graph_Change, 2, UI_Color_White, 18, 8, 3,  Left_Title_List_X, 530, "FOLW");//跟随模式指示，粉色为关，绿色为开
		UI_PushUp_String(&UI_String, get_robot_id());
	}
		
    //	if (UI_PushUp_Counter % 171 == 0) //动态UI绘制 左键点击数量
    //	{
    //		UI_Draw_Float(&UI_Graph1.Graphic[0], "203", UI_Graph_Change, 2, UI_Color_White, 22, 0, 3, 1500, 632, Shoot_Num);
    //		UI_PushUp_Graphs(1, &UI_Graph1, get_robot_id());
    //	}
    //		if(UI_PushUp_Counter % 21 == 0) //动态UI更新 字符串1
    //		{
    //			if(UI_fric_is_on == 1)
    //			{
    //				if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==0)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre  NO");
    //				}
    //				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==0)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre  NO");
    //				}
    //				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==0)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre  NO");
    //				}
    //				else if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==1)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre YES");
    //				}
    //				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==1)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre YES");
    //				}
    //				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==1)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre YES");
    //				}
    //
    //				else if(autoaim_mode==0x03)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nXFu\n        \n       ");
    //				}
    //				else if(autoaim_mode==0x04)
    //				{
    //					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nDFu\n        \n       ");
    //				}
    //			}
    //			//if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8+4+9+8, 3,  100, 700, "Fric OFF\n   \n        \n       ");
    //			UI_PushUp_String(&UI_String, get_robot_id());
    //		}
    //if(UI_PushUp_Counter % 351 == 0)
    //{UI_Draw_String(&UI_String1.String, "305", UI_Graph_Add, 2, UI_Color_Green, 22, 12, 3,  10, 700, "vision_mode");//摩擦轮是否开启
    //		//UI_Draw_String(&UI_String1.String, "305", UI_Graph_Add, 2,  UI_Color_Yellow,  22, 11, 3,10,500,"vision_mode");
    //
    //UI_PushUp_String(&UI_String1, get_robot_id());}
    if(UI_PushUp_Counter % 50 == 0)  //动态UI更新 图形
    {
        /* Pitch轴当前角度 */
        extern float pich_angle;
        int IntIze_Pitch_Angle;
        IntIze_Pitch_Angle = pich_angle * 100;
        if (IntIze_Pitch_Angle > -1900 && IntIze_Pitch_Angle < 2800) //未触发角度保护
            UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Green, 22, 3, 3, 1000, 632, pich_angle);
        else if (IntIze_Pitch_Angle > -2100 && IntIze_Pitch_Angle < 2850)
            UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 4, 1000, 632, pich_angle);
        else UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Purple, 22, 5, 5, 1000, 632, pich_angle);

        /* 超级电容容量 */
        extern int Supercap_Connection_Status;
		if (Relay_Set_State == Supercap_Power_Supply)
		{
			if (Supercap_Connection_Status)
				UI_Draw_Float(&UI_Graph2.Graphic[1], "205", UI_Graph_Change, 1, UI_Color_White, 18, 0, 3, 1700, 692, supercap_volt);
			else UI_Draw_Float(&UI_Graph2.Graphic[1], "205", UI_Graph_Change, 1, UI_Color_Purple, 22, 2, 5, 1700, 692, 0.00);
		}
		else 
			UI_Draw_Float(&UI_Graph2.Graphic[1], "205", UI_Graph_Change, 1, UI_Color_White, 18, 0, 0, 1700, 692, supercap_volt); //线宽为0，不显示
        UI_PushUp_Graphs(2, &UI_Graph2, get_robot_id());
    }
}

