#include "PID.h"
// PID�Ż����ں�������
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

/**
 * @brief          PID��ʼ��   PID initialize
 * @param[in]      PID�ṹ��   PID structure
 * @param[in]      ��
 * @retval         ���ؿ�      null
 */
void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Ref = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    // ���ٻ��ֲ���
    // coefficient of changing integration rate
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;

    pid->Derivative_LPF_RC = derivative_lpf_rc;

    // ��С������ȡ�ź�΢�ֳ�ʼ��
    // differential signal is distilled by OLS
    pid->OLS_Order = ols_order;
    OLS_Init(&pid->OLS, ols_order);

    // DWT��ʱ��������������
    // reset DWT Timer count counter
    pid->DWT_CNT = 0;

    // ����PID�Ż�����
    pid->Improve = improve;

    // ����PID�쳣���� Ŀǰ�����������ת����
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**
 * @brief          PID����
 * @param[in]      PID�ṹ��
 * @param[in]      ����ֵ
 * @param[in]      ����ֵ
 * @retval         ���ؿ�
 */
float PID_Calculate(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT((uint32_t*)&pid->DWT_CNT);

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);

    if (abs(pid->Err) > pid->DeadBand)
    {
        if (pid->FuzzyRule == NULL)
        {
            pid->Pout = pid->Kp * pid->Err;
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        }
        else
        {
            pid->Pout = (pid->Kp + pid->FuzzyRule->KpFuzzy) * pid->Err;
            pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Err - pid->Last_Err) / pid->dt;
        }

        if (pid->User_Func2_f != NULL)
            pid->User_Func2_f(pid);

        // ���λ���
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // ���ٻ���
        if (pid->Improve & ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // ΢������
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // ΢���˲���
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);
        // �����޷�
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        // ����˲�
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        // ����޷�
        f_Output_Limit(pid);

        // �޹ؽ�Ҫ
        f_Proportion_Limit(pid);
    }

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

static void f_Trapezoid_Intergral(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
    else
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // ���ֳ��ۻ�����
        // Integral still increasing
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // ���ֳ��ۻ�����
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
    {
        if (pid->OLS_Order > 2)
            pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        if (pid->OLS_Order > 2)
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

// PID ERRORHandle Function
static void f_PID_ErrorHandle(PID_t *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*************************** FEEDFORWARD CONTROL *****************************/
/**
 * @brief          ǰ�����Ƴ�ʼ��
 * @param[in]      ǰ�����ƽṹ��
 * @param[in]      ��
 * @retval         ���ؿ�
 */
void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order)
{
    ffc->MaxOut = max_out;

    // ����ǰ������������ ���ǰ�����ƽṹ�嶨��
    // set parameters of feed-forward controller (see struct definition)
    if (c != NULL && ffc != NULL)
    {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    }
    else
    {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->MaxOut = 0;
    }

    ffc->LPF_RC = lpf_rc;

    // ��С������ȡ�ź�΢�ֳ�ʼ��
    // differential signal is distilled by OLS
    ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
    ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2)
        OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
    if (ref_ddot_ols_order > 2)
        OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

    ffc->DWT_CNT = 0;

    ffc->Output = 0;
}

/**
 * @brief          PID����
 * @param[in]      PID�ṹ��
 * @param[in]      ����ֵ
 * @param[in]      ����ֵ
 * @retval         ���ؿ�
 */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
    ffc->dt = DWT_GetDeltaT((uint32_t*)&ffc->DWT_CNT);

    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) +
               ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);

    // ����һ�׵���
    // calculate first derivative
    if (ffc->Ref_dot_OLS_Order > 2)
        ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
    else
        ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;

    // ������׵���
    // calculate second derivative
    if (ffc->Ref_ddot_OLS_Order > 2)
        ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
    else
        ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;

    // ����ǰ���������
    // calculate feed-forward controller output
    ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;

    ffc->Output = float_constrain(ffc->Output, -ffc->MaxOut, ffc->MaxOut);

    ffc->Last_Ref = ffc->Ref;
    ffc->Last_Ref_dot = ffc->Ref_dot;

    return ffc->Output;
}