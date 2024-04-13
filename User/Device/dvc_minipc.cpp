/**
 * @file dvc_minipc.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 迷你主机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright ustc-robowalker (c) 2023
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"
#include "crt_gimbal.h"
#include "drv_math.h"

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object *__USB_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
    USB_Manage_Object = __USB_Manage_Object;
    Frame_Header = __frame_header;
    Frame_Rear = __frame_rear;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
    // memcpy(&Data_NUC_To_MCU, ((Struct_MiniPC_USB_Data *)USB_Manage_Object->Rx_Buffer)->Data, sizeof(Struct_MiniPC_Rx_Data));
    memcpy(&Pack_Rx, (Pack_rx_t *)USB_Manage_Object->Rx_Buffer, USB_Manage_Object->Rx_Buffer_Length);

    float tmp_yaw, tmp_pitch;

    Self_aim(Pack_Rx.target_x, Pack_Rx.target_y, Pack_Rx.target_z+0.06, &Rx_Angle_Yaw, &Rx_Angle_Pitch, &Distance);

    //    Rx_Angle_Yaw = meanFilter(tmp_yaw);
    //    Rx_Angle_Pitch = meanFilter(tmp_pitch);
    // if(Pack_Rx.hander!=0xA5) memset(&Pack_Rx,0,USB_Manage_Object->Rx_Buffer_Length);

    memset(USB_Manage_Object->Rx_Buffer, 0, USB_Manage_Object->Rx_Buffer_Length);
}

/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */
void Class_MiniPC::Output()
{
    Pack_Tx.hander = Frame_Header;

    // 根据referee判断红蓝方
    if (Referee->Get_ID() >= 101)
        Pack_Tx.detect_color = 101; // 蓝方
    else
        Pack_Tx.detect_color = 0; // 红方

    Pack_Tx.target_id = 0x01;
    Pack_Tx.Game_Status_Stage = Referee->Get_Game_Stage();
    Pack_Tx.roll = Tx_Angle_Roll;
    Pack_Tx.pitch = Tx_Angle_Pitch;
    Pack_Tx.yaw = Tx_Angle_Yaw;
    Pack_Tx.crc16 = 0xffff;
    memcpy(USB_Manage_Object->Tx_Buffer, &Pack_Tx, sizeof(Pack_Tx));
    Append_CRC16_Check_Sum(USB_Manage_Object->Tx_Buffer, sizeof(Pack_Tx));
    USB_Manage_Object->Tx_Buffer_Length = sizeof(Pack_Tx);
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
    Transform_Angle_Tx();
    Output();
}

/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
    // 滑动窗口, 判断迷你主机是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        // 迷你主机断开连接
        MiniPC_Status = MiniPC_Status_DISABLE;
    }
    else
    {
        // 迷你主机保持连接
        MiniPC_Status = MiniPC_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t ch_data;

    if (pchMessage == NULL)
        return 0xFFFF;
    while (dwLength--)
    {
        ch_data = *pchMessage++;
        wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
    }

    return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t w_expected = 0;

    if ((pchMessage == NULL) || (dwLength <= 2))
        return false;

    w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
    return (
        (w_expected & 0xff) == pchMessage[dwLength - 2] &&
        ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t w_crc = 0;

    if ((pchMessage == NULL) || (dwLength <= 2))
        return;

    w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

    pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

/**
 * 计算给定向量的偏航角（yaw）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float Class_MiniPC::calc_yaw(float x, float y, float z)
{
    // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
    if (x == 0)
    {
        return Gimbal_Yaw_Motor_GM6020->Get_True_Angle_Yaw();
    }

    float yaw = atan2f(y, x);

    // 将弧度制的偏航角转换为角度制
    yaw = (yaw * 180 / 3.1415926); // 向左为正，向右为负

    return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float Class_MiniPC::calc_distance(float x, float y, float z)
{
    // 计算各分量的平方和，并取其平方根得到欧几里德距离
    float distance = sqrtf(x * x + y * y + z * z);

    return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */

uint8_t temp = 0;
float dz;
float Class_MiniPC::calc_pitch(float x, float y, float z)
{
    // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
    if (x == 0 || z == 0)
    {
        return IMU->Get_Angle_Pitch();
    }

    if (isnan(x) || isnan(y))
    {
        temp++;
    }

    //    float pitch = atan2f(z, x);
    // float pitch = atan2f(z, Math_Abs(x));
    // if(isnan(pitch))
    // {
    //     temp++;
    // }
    
    float temp_hight;
    temp_hight = z;
    float pitch;
	pitch = atan2f(z, sqrtf(x*x+y*y));
//    // 使用重力加速度模型迭代更新俯仰角
//    for (size_t i = 0; i < 20; i++)
//    {
//        pitch = atan2f(temp_hight, Math_Abs(x));
//        float v_x = bullet_v * cosf(pitch);
//        if (v_x == 0)
//        {
//            temp++;
//        }
//        float v_y = bullet_v * sinf(pitch);

//        float t = sqrtf(x * x) / v_x;
//        float h = v_y * t - 0.5 * g * t * t;
//        dz = z - h;
//        
//        temp_hight += 0.5 * dz;

//        if (fabsf(dz) < 0.01)
//        {
//            break;
//        }

//        // 根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
//        //   pitch += asinf(dz / calc_distance(x, y, z));
//    }

    // 将弧度制的俯仰角转换为角度制
    pitch = (pitch * 180 / 3.1415926); //
    
    if(pitch<-40)
    {
	    temp++;
    }

    return pitch;
}
/**
 * 计算计算yaw，pitch
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */
void Class_MiniPC::Self_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{
    *yaw = calc_yaw(x, y, z);
    *pitch = calc_pitch(x, y, z);
    *distance = calc_distance(x, y, z);
}

float Class_MiniPC::meanFilter(float input)
{
    static float buffer[5] = {0};
    static uint64_t index = 0;
    float sum = 0;

    // Replace the oldest value with the new input value
    buffer[index] = input;

    // Increment the index, wrapping around to the start of the array if necessary
    index = (index + 1) % 5;

    // Calculate the sum of the buffer's values
    for (int i = 0; i < 5; i++)
    {
        sum += buffer[i];
    }

    // Return the mean of the buffer's values
    return sum / 5.0;
}
/************************ copyright(c) ustc-robowalker **************************/
