#include "main.h"

/*Sample varible*/
#define SAMPLE_PERIOD 0.00005

// 角度生成参数
THETA_REGS U_theta;   // 电压环theta
THETA_REGS I_theta;   // 电流环theta
THETA_REGS G_theta;   // PLL电网theta
THETA_REGS VSG_theta; // VSG电网theta

/*matlab_varible---->C_varible*/
Sample Vol_Vs;                // 机端电压采样变量
Sample Curr_Is;               // 机端电流采样变量
Sample Curr_Iabc;             // 逆变器输出电流采样变量
Sample Vol_Vg;                // 电网电压采样变量
Sample Curr_Ic;               // 滤波电容电流采样变量
float32 Sample_Pe, Sample_Qe; // 电磁功率采样变量

/*2025年3月9日VSG控制时将采样变量升级为结构体变量，如上

float32 Sample_vol_As, Sample_vol_Bs, Sample_vol_Cs;    // 机端电压采样变量
float32 Sample_curr_As, Sample_curr_Bs, Sample_curr_Cs; // 机端电流采样变量
float32 Sample_curr_A, Sample_curr_B, Sample_curr_C;    // 逆变器输出电流采样变量
float32 Sample_Grid_A, Sample_Grid_B, Sample_Grid_C;    // 电网电压采样变量
float32 Sample_curr_Ca, Sample_curr_Cb, Sample_curr_Cc; // 滤波电容电流采样变量
float32 Sample_Pe, Sample_Qe;                           // 电磁功率采样变量
*/

// 参考值
float32 Vref, Iref, Vdc;

/// @brief sin&cos calculate
/// @param theta
void sin_cos_cal(THETA_REGS *p)
{
    p->cos_theta = cosf(p->theta); // cos(θ)
    p->sin_theta = sinf(p->theta); // sin(θ)
    p->cos_120 = -0.5;             // cos(120°)
    p->sin_120 = 0.8660254;        // sin(120°)

    p->cos_2theta = (p->cos_theta * p->cos_theta) - (p->sin_theta * p->sin_theta); // cos(2θ)
    p->sin_2theta = (p->sin_theta * p->cos_theta) * 2;                             // sin(2θ)

    p->cos_theta_p_120 = (p->cos_theta * p->cos_120) - (p->sin_theta * p->sin_120); // cos(θ+120°)
    p->cos_theta_m_120 = (p->cos_theta * p->cos_120) + (p->sin_theta * p->sin_120); // cos(θ-120°)

    p->sin_theta_p_120 = (p->sin_theta * p->cos_120) + (p->cos_theta * p->sin_120); // sin(θ+120°)
    p->sin_theta_m_120 = (p->sin_theta * p->cos_120) - (p->cos_theta * p->sin_120); // sin(θ-120°)
}

/// @brief angle variable initialization
void THETA_REGS_VAR_INIT(THETA_REGS *p)
{
    p->theta = 0;
    p->cos_theta = 0;
    p->sin_theta = 0;
    p->cos_2theta = 0;
    p->sin_2theta = 0;

    p->cos_theta_p_120 = 0;
    p->cos_theta_m_120 = 0;
    p->sin_theta_p_120 = 0;
    p->sin_theta_m_120 = 0;
    p->cos_120 = 0;
    p->sin_120 = 0;
}

// PID变量初始化
void PID_VAR_INIT(PID *p)
{
    p->Kp = 0;
    p->Ki = 0;
    p->Kc = 0;
    p->ref = 0;
    p->fdb = 0;
    p->err = 0;
    p->ui = 0;
    p->up = 0;
    p->upresat = 0;
    p->uo = 0;
    p->upper_limit = 0;
    p->lower_limit = 0;
}

// RAMP_REFERENCE变量初始化
void RAMP_VAR_INIT(RAMP_REFERENCE *p)
{
    p->Given = 0;
    p->count = 0;
    p->delta = 0;
    p->length = 0;
    p->output = 0;
}

// 正序相反变换
void dq2abc(DQ2ABC *p, THETA_REGS *q)
{
#define c1 0.666666667 // 2/3
    p->a = q->cos_theta * p->d - q->sin_theta * p->q;
    p->b = q->cos_theta_m_120 * p->d - q->sin_theta_m_120 * p->q;
    p->c = q->cos_theta_p_120 * p->d - q->sin_theta_p_120 * p->q;

    //	 p->b= 2;//q->cos_theta_m_120*p->d- q->sin_theta_m_120*p->q;
    //	 p->c= 3;//q->cos_theta_p_120*p->d- q->sin_theta_p_120*p->q;
}

/// @brief abc -> alpha,beta (constant amplitude transform)
/// @brief Clark变换（恒幅值）
void Clark(CLARK_REGS *p)
{
    p->alpha = p->a; // 2/3(ua - ub/2  -uc/2)
    p->beta = (p->b - p->c) / sqrt(3);
}

/// @brief alpha,beta -> abc
void iClark(ICLARK_REGS *p)
{
    p->a = p->alpha;
    p->b = (p->alpha * (-0.5)) + (p->beta * sqrt(3) / 2);
    p->c = (p->alpha * (-0.5)) - (p->beta * sqrt(3) / 2);
}

/// @brief abc -> alpha,beta(alpha 滞后a相90°)
/// @param p
void Clark_d90A(CLARK_REGS *p)
{
    p->alpha = (p->c - p->b) / sqrt(3);
    p->beta = p->a; // 2/3(ua - ub/2  -uc/2)
}

/// @brief alpha,beta -> d,q (constant amplitude transform)
void Park(PARK_REGS *p, THETA_REGS *q)
{
    p->d = (p->alpha * q->cos_theta) + (p->beta * q->sin_theta);
    p->q = (-p->alpha * q->sin_theta) + (p->beta * q->cos_theta);
}

/// @brief d,q -> alpha,beta
void iPark(IPARK_REGS *p, THETA_REGS *q)
{
    p->alpha = (p->d * q->cos_theta) + (-p->q * q->sin_theta);
    p->beta = (p->d * q->sin_theta) + (p->q * q->cos_theta);
}

/// @brief alpha,beta-->d,q(alpha 滞后a相90°)
/// @param p
/// @param q
void Park_d90A(PARK_REGS *p, THETA_REGS *q)
{
    p->d = (p->alpha * q->sin_theta) - (p->beta * q->cos_theta);
    p->q = (p->alpha * q->cos_theta) + (p->beta * q->sin_theta);
}

/// @brief Universal_Coordinate_Transformation
void abc2dq(Sample *var, THETA_REGS *p)
{
    PARK_REGS GPark;
    CLARK_REGS GClark;

    GClark.a = var->a;
    GClark.b = var->b;
    GClark.c = var->c;

    Clark(&GClark);

    GPark.alpha = GClark.alpha;
    GPark.beta = GClark.beta;

    Park(&GPark, p);

    var->d = GPark.d;
    var->q = GPark.q;
}

/// @brief coordinate transformation
void INV_XY_CAL(THETA_REGS *p)
{

    sin_cos_cal(p);

    abc2dq(&Vol_Vs, p);    // 机端电压坐标变换
    abc2dq(&Curr_Iabc, p); // 逆变器输出电流坐标变换
    abc2dq(&Curr_Is, p);   // 机端电流坐标变换
}
// {
//     abc2dq(&Vol_Vs, &I_theta);    // 机端电压坐标变换
//     abc2dq(&Curr_Is, &I_theta);   // 机端电流坐标变换
//     abc2dq(&Curr_Iabc, &I_theta); // 逆变器输出电流坐标变换
// }

/// @brief Ramp Given(given->目标值；delta->变换率；length->变换时间）
/// @param ramp_reference(v)
void Ramp_Given(RAMP_REFERENCE *v)
{
    v->count = v->count + 1;
    if (v->count >= v->length)
    {
        v->count = 0;
        if (v->output < v->Given)
        {
            v->output = v->output + v->delta;
            if (v->output > v->Given)
            {
                v->output = v->Given;
            }
        }
        else if (v->output > v->Given)
        {
            v->output = v->output - v->delta;
            if (v->output < v->Given)
            {
                v->output = v->Given;
            }
        }
    }
}

/// @brief PID calculation
/// @param p
void Pid_calculation(PID *p)
{
    // 误差
    p->err = p->ref - p->fdb;

    // 比例
    p->up = p->err * p->Kp;
    // 给比例限幅
    // if (p->up >= p->upper_limit)
    // {
    //     p->up = p->upper_limit;
    // }
    // else if (p->up <= p->lower_limit)
    // {
    //     p->up = p->lower_limit;
    // }

    // 积分
    p->ui = p->ui + ((p->err * p->Ki) * SAMPLE_PERIOD);
    // 给积分限幅
    if (p->ui >= p->upper_limit)
    {
        p->ui = p->upper_limit;
    }
    else if (p->ui <= p->lower_limit)
    {
        p->ui = p->lower_limit;
    }

    // 求和
    p->upresat = p->up + p->ui;
    // 给输出限幅
    if (p->upresat >= p->upper_limit)
    {
        p->uo = p->upper_limit;
    }
    else if (p->upresat <= p->lower_limit)
    {
        p->uo = p->lower_limit;
    }
    else
    {
        p->uo = p->upresat;
    }
}
