#ifndef LIBRARY_H_
#define LIBRARY_H_

#include "main.h"

// Definition of 16-bit and 32-bit signed/unsigned integers and decimals:
typedef int int16;
typedef long int32;
typedef long long int64;
typedef unsigned int Uint16;
typedef unsigned long Uint32;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;

/*角度结构体*/
typedef struct
{
    float32 theta;
    float32 cos_theta;
    float32 sin_theta;
    float32 cos_2theta;
    float32 sin_2theta;

    float32 cos_theta_p_120;
    float32 cos_theta_m_120;
    float32 sin_theta_p_120;
    float32 sin_theta_m_120;
    float32 cos_120;
    float32 sin_120;
} THETA_REGS;
#define THETA_REGS_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

typedef struct
{
    float32 a;
    float32 b;
    float32 c;
    float32 d;
    float32 q;
} Sample;

// 滤波器结构体（保存参数和状态）
typedef struct
{
    float alpha;  // 滤波系数α
    float y_prev; // 上一次的输出值
    float dt;     // 采样时间（秒）
    float fc;     // 截止频率（Hz）
} LowPassFilter;

extern THETA_REGS U_theta;
extern THETA_REGS I_theta;
extern THETA_REGS G_theta;
extern THETA_REGS VSG_theta;

/*matlab_varible---->C_varible*/
extern Sample Vol_Vs;    // 机端电压采样变量
extern Sample Curr_Is;   // 机端电流采样变量
extern Sample Curr_Iabc; // 逆变器输出电流采样变量
extern Sample Vol_Vg;    // 电网电压采样变量
extern Sample Curr_Ic;   // 滤波电容电流采样变量
extern Sample SVPWM_var;
extern float Sample_Pe, Sample_Qe; // 电磁功率采样变量

extern LowPassFilter filter_P; // 滤波器变量
extern LowPassFilter filter_V;

/*****************SVPWM变量************************/
extern float t_a, t_b, t_c; // 原始输入三相信号
extern float t_al, t_be;    // 坐标变换变量
extern int A, B, C, n;      // 扇区判断变量
extern float T1, T2, T0;    // 时间计算变量
extern float vta, vtb, vtc; // 参考调制波

extern float32 Vref, Iref, Vdc;

extern void sin_cos_cal(THETA_REGS *p);             // sin&cos calculate
extern void THETA_REGS_VAR_INIT(THETA_REGS *p);     // angle variable initialization
extern void PID_VAR_INIT(PID *p);                   // PID变量初始化
extern void RAMP_VAR_INIT(RAMP_REFERENCE *p);       // RAMP_REFERENCE变量初始化
extern void INV_XY_CAL(THETA_REGS *p);              // coordinate transformation
extern void dq2abc(DQ2ABC *p, THETA_REGS *q);       // 正序相反变换
extern void Clark(CLARK_REGS *p);                   // Clark变换（恒幅值）abc -> alpha,beta (constant amplitude transform)
extern void iClark(ICLARK_REGS *p);                 // alpha,beta -> abc
extern void Clark_d90A(CLARK_REGS *p);              // abc -> alpha,beta(alpha 滞后a相90°)
extern void Park(PARK_REGS *p, THETA_REGS *q);      // alpha,beta -> d,q (constant amplitude transform)
extern void iPark(IPARK_REGS *p, THETA_REGS *q);    // d,q -> alpha,beta
extern void Park_d90A(PARK_REGS *p, THETA_REGS *q); // alpha,beta-->d,q(alpha 滞后a相90°)
extern void abc2dq(Sample *var, THETA_REGS *p);     // abc ->d,q(constant amplitude transform)
extern void Ramp_Given(RAMP_REFERENCE *v);          // Ramp Given(given->目标值；delta->变换率；length->变换时间）
extern void Pid_calculation(PID *p);                // PID calculation
extern void PQ_Calculation(Sample *v, Sample *i);
extern void init_low_pass_filter(LowPassFilter *filter, float dt, float fc); // 初始化滤波器
extern void update_low_pass_filter(LowPassFilter *filter, float input);      // 更新滤波器状态，返回当前输出
extern void SVPWM_Control(Sample *in_var);

#endif /* LIBRARY_H_ */
