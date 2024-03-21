/**
 * @file alg_power_limit.cpp
 * @author �W�W�W (850184312@qq.com)
 * @brief ��������
 * @version 1.1
 * @date 2024-03-21 0.1 24��������
 *
 * @copyright Copyright (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_power_limit.h"
#include "dvc_djimotor.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief ��ʼ������
 *
 * @param __K1 ����K1��ֵ��Ĭ��Ϊ1.23e-07f
 * @param __K2 ����K2��ֵ��Ĭ��Ϊ1.453e-07f
 * @param __alpha ����alpha��ֵ��Ĭ��Ϊ4.081f
 * @param toque_coefficient Ť��ϵ���ȵ�ֵ��Ĭ��Ϊ1.99688994e-6f
 */
void Class_Power_Limit::Parameter_Init(float __K1 , float __K2 , float __alpha , float __toque_coefficient)
{
    k1 = __K1;
    k2 = __K2;
    Alpha = __alpha;
    Toque_Coefficient = __toque_coefficient;
}

/**
 * @brief ��ȡ���Ť�ص���
 *
 * @param num ������
 * @return float ���Ť�ص���
 */
float Class_Power_Limit::Get_Torque_Current(uint8_t num)
{
    return Torque_Current[num];
}

/**
 * @brief ��ʱ�����ڵ���ص�����
 *
 */
void Class_Power_Limit::TIM_Adjust_PeriodElapsedCallback()
{
	//ÿ�����ڵ��ܹ���Ԥ��������
	Total_Predict_Power = 0;
	//�����ܹ������������ϵ��K
	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		Predict_Power[i] =  Toque_Coefficient * Torque_Current[i] * Omega[i] * RAD_TO_RPM+
							k2 * (Omega[i] * RAD_TO_RPM)* (Omega[i] * RAD_TO_RPM) +
							k1 * Torque_Current[i] * Torque_Current[i] + 
                            Alpha;

		if (Predict_Power[i] < 0) // negative power not included (transitory)
			continue;
		Total_Predict_Power += Predict_Power[i];
	}

    //������������
	if (Total_Predict_Power > Total_Power_Limit) // determine if larger than max power
	{
		//��������ϵ��k
		Power_Scale = Total_Power_Limit / Total_Predict_Power;
		for (uint8_t i = 0; i < 4; i++)
		{
			Scaled_Give_Power[i] = Predict_Power[i] * Power_Scale; // ��ø������������Ĺ�������
			if (Scaled_Give_Power[i] < 0)
			{
				continue;
			}

            //���̱��� equation_b equation_c
			equation_b = Toque_Coefficient * Omega[i] * RAD_TO_RPM;
			equation_c = k2 * Omega[i] * Omega[i] - Scaled_Give_Power[i] + Alpha;

			if (Torque_Current[i] > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-equation_b + sqrt(equation_b * equation_b - 4 * k1 * equation_c)) / (2 * k1);
				if (temp > 16000)
				{
					Torque_Current[i] = 16000;
				}
				else
					Torque_Current[i] = temp;
			}
			else
			{
				float temp = (-equation_b - sqrt(equation_b * equation_b - 4 * k1 * equation_c)) / (2 * k1);
				if (temp < -16000)
				{
					Torque_Current[i] = -16000;
				}
				else
					Torque_Current[i] = temp;
			}
		}
	}
}

/**
 * @brief �趨����������
 *
 */
void Class_Power_Limit::Output(Class_DJI_Motor_C620 (&Motor)[4])
{
    for(int i=0;i<4;i++)
	{
        Motor[i].CAN_Tx_Data[0] = (int16_t)Torque_Current[i] >> 8;
        Motor[i].CAN_Tx_Data[1] = (int16_t)Torque_Current[i];
    }
}

/**
 * @brief �趨�ĸ�����Ŀ��Ƶ����͵�ǰ���ٶ�
 *
 */
void Class_Power_Limit::Set_Motor(Class_DJI_Motor_C620 (&Motor)[4])
{
    for(int i=0;i<4;i++)
    {
        Torque_Current[i] = Motor[i].Get_Out();
        Omega[i] = Motor[i].Get_Now_Omega();
    }
}

/* Function prototypes -------------------------------------------------------*/


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
