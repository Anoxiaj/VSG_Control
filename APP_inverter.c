#include "main.h"

int jishu = 0;
float theta_50Hz, PLL_theta;
#define PIE 3.1415926535897932384626433832795
const float PIEx2 = 6.283185307179586476925286766559;
const float PIEx100 = 314.15926535897932384626433832795;

float waveA, waveB, waveC;

float test1, test2, test3, test4;
float test5, test6, test7, test8;

float back_d, back_q; // 双闭环传递参数

PID Ud_pid;
PID Uq_pid;
PID Id_pid;
PID Iq_pid;
PID PLL_pid;
PID VSG_pid;

RAMP_REFERENCE Ud_ramp;
RAMP_REFERENCE Uq_ramp;
RAMP_REFERENCE Id_ramp;
RAMP_REFERENCE Iq_ramp;

VSG_Params vsg;

/// @brief Generate the angle of system control
void THETA_GENERATE(void)
{
    /* theta=wt , w=2pie f */
    theta_50Hz = theta_50Hz + delta_time * PIEx100; // 0.0001:interrupt time 10K
    theta_50Hz = theta_50Hz > PIEx2 ? (theta_50Hz - PIEx2) : theta_50Hz;
    theta_50Hz = theta_50Hz < 0 ? (theta_50Hz + PIEx2) : theta_50Hz;

    U_theta.theta = theta_50Hz;
    I_theta.theta = theta_50Hz;

    // test5 = 60 * U_theta.theta;
    // test6 = Vol_Vs.a;
}

void OPEN_LOOP(float Modulation)
{
    DQ2ABC v;
    v.d = Modulation;
    v.q = 0;
    dq2abc(&v, &I_theta);

    SVPWM_var.a = v.a;
    SVPWM_var.b = v.b;
    SVPWM_var.c = v.c;

    waveA = (v.a + 1) / 2;
    waveB = (v.b + 1) / 2;
    waveC = (v.c + 1) / 2;
}

/// @brief PID Voltage Closed Loop
/// @param V_ref
void VOLTAGE_CLOSED_LOOP(float V_ref, float V_q, float d_feedback, float q_feedback)
{
    IPARK_REGS UiPark;
    ICLARK_REGS UiClark;

    // /*d轴斜坡给定*/
    // Ud_ramp.Given = V_ref;
    // Ud_ramp.delta = 1;
    // Ud_ramp.length = 1;

    // Ramp_Given(&Ud_ramp);
    // Ud_pid.ref = Ud_ramp.output;
    Ud_pid.ref = V_ref;
    Ud_pid.fdb = d_feedback;

    /*d轴PID计算：单闭环时Kp=0.05 Ki=5*/ // 100: 2 300 ; 150:0.1 50
    Ud_pid.Kp = U_Kp_parameter;
    Ud_pid.Ki = U_Ki_parameter;
#if switch_loop
    /*单闭环时限幅为+-1*/
    Ud_pid.upper_limit = 1;  // d轴PID限幅
    Ud_pid.lower_limit = -1; // d轴PID限幅
#else
    Ud_pid.upper_limit = 1000000;  // d轴PID限幅
    Ud_pid.lower_limit = -1000000; // d轴PID限幅
#endif

    Pid_calculation(&Ud_pid); // 计算出d轴pid输出

    // /*q轴斜坡给定*/
    // Uq_ramp.Given = 0;
    // Uq_ramp.delta = 1;
    // Uq_ramp.length = 1;

    // Ramp_Given(&Uq_ramp);
    // Uq_pid.ref = Uq_ramp.output;
    Uq_pid.ref = 0;
    Uq_pid.fdb = q_feedback;

    /*q轴PID计算*/
    Uq_pid.Kp = U_Kp_parameter;
    Uq_pid.Ki = U_Ki_parameter;

    Uq_pid.upper_limit = 1000000;  // d轴PID限幅
    Uq_pid.lower_limit = -1000000; // d轴PID限幅

    Pid_calculation(&Uq_pid); // 计算出q轴pid输出

    /*把pid计算出的dq输出值赋给park反变换的dq*/
    UiPark.d = Ud_pid.uo;
    UiPark.q = Uq_pid.uo;

    iPark(&UiPark, &U_theta); // Park反变换

    /*把Park反变换的α和β赋给Clark反变换的α和β*/
    UiClark.alpha = UiPark.alpha;
    UiClark.beta = UiPark.beta;

    iClark(&UiClark); // Clark反变换
#if switch_loop
    /*电压单闭环模块*/
    /*对调制波进行归一化（SPWM的相电压最大值为(Vdc/2)*/
    waveA = (UiClark.a + 1) / 2;
    waveB = (UiClark.b + 1) / 2;
    waveC = (UiClark.c + 1) / 2;
#else
    /*双闭环模块*/
    back_d = Ud_pid.uo;
    back_q = Uq_pid.uo;
#endif
    // test5 = V_ref;
    test5 = d_feedback;
    // test7 = V_q;
    test6 = q_feedback;
}

/// @brief PID Current Closed Loop
/// @param I_ref
/// @param I_q
/// @param d_feedback
/// @param q_feedback
/// @param Theta
void CURRENT_CLOSED_LOOP(float I_ref, float I_q, float d_feedback, float q_feedback)
{
    IPARK_REGS IiPark;
    ICLARK_REGS IiClark;

    // /*d轴斜坡给定*/
    // Id_ramp.Given = I_ref;
    // Id_ramp.delta = 1;
    // Id_ramp.length = 1;

    // Ramp_Given(&Id_ramp);
    // Id_pid.ref = Id_ramp.output;
    Id_pid.ref = I_ref;
    Id_pid.fdb = d_feedback;

    /*d轴PID计算*/
    Id_pid.Kp = I_Kp_parameter;
    Id_pid.Ki = I_Ki_parameter;

    Id_pid.upper_limit = 1;  // d轴PID限幅
    Id_pid.lower_limit = -1; // d轴PID限幅

    Pid_calculation(&Id_pid); // 计算出d轴pid输出

    // /*q轴斜坡给定*/
    // Iq_ramp.Given = I_q;
    // Iq_ramp.delta = 1;
    // Iq_ramp.length = 1;

    // Ramp_Given(&Iq_ramp);
    // Iq_pid.ref = Iq_ramp.output;
    Iq_pid.ref = I_q;
    Iq_pid.fdb = q_feedback;

    /*q轴PID计算*/
    Iq_pid.Kp = I_Kp_parameter;
    Iq_pid.Ki = I_Ki_parameter;

    Iq_pid.upper_limit = 1;  // d轴PID限幅
    Iq_pid.lower_limit = -1; // d轴PID限幅

    Pid_calculation(&Iq_pid); // 计算出q轴pid输出

    /*把pid计算出的dq输出值赋给park反变换的dq*/
    IiPark.d = Id_pid.uo;
    IiPark.q = Iq_pid.uo;

    // IiPark.d = Id_pid.uo - q_feedback * PIEx100 * inv_params_L1; // 解耦：w=PIEx100
    // IiPark.q = Iq_pid.uo + d_feedback * PIEx100 * inv_params_L1;

    // IiPark.d = Id_pid.uo + Vol_Vs.d; // 电网电压前馈
    // IiPark.q = Iq_pid.uo + Vol_Vs.q;

    iPark(&IiPark, &VSG_theta); // Park反变换

    /*把Park反变换的α和β赋给Clark反变换的α和β*/
    IiClark.alpha = IiPark.alpha;
    IiClark.beta = IiPark.beta;

    iClark(&IiClark); // Clark反变换

    /*有源阻尼测试代码*/
    // if (jishu > 3200) // 16k下对应0.2s处
    // {
    IiClark.a = IiClark.a - Curr_Ic.a * damping;
    IiClark.b = IiClark.b - Curr_Ic.b * damping;
    IiClark.c = IiClark.c - Curr_Ic.c * damping;
    // }

    SVPWM_var.a = IiClark.a;
    SVPWM_var.b = IiClark.b;
    SVPWM_var.c = IiClark.c;

    /*对调制波进行归一化（SPWM的相电压最大值为(Vdc/2)*/
    waveA = (IiClark.a + 1) / 2;
    waveB = (IiClark.b + 1) / 2;
    waveC = (IiClark.c + 1) / 2;

    jishu++;

    // test1 = I_ref;
    test7 = d_feedback;
    // test3 = I_q;
    test8 = q_feedback;
}

/// @brief Phase-Locked Loop (PLL) control function
void PHASE_LOCKED_LOOP(void)
{
    CLARK_REGS GClark;
    PARK_REGS GPark;

    // 使用时需要把Sample_Grid_A,B,C赋值给GClark.a,b,c
    GClark.a = Vol_Vs.a;
    GClark.b = Vol_Vs.b;
    GClark.c = Vol_Vs.c;

    Clark(&GClark);

    GPark.alpha = GClark.alpha;
    GPark.beta = GClark.beta;

    sin_cos_cal(&G_theta);

    /*
    锁相环锁角度，与给定无关，只与q轴与A轴的关系有关
    q轴与A轴重合时，锁的是Cos型：Park(&GPark, &G_theta);
    q轴滞后A轴90°时，锁的是Sin型：Park_d90A(&GPark, &G_theta);
    */
    Park(&GPark, &G_theta);

    PLL_pid.ref = 0;
    PLL_pid.fdb = GPark.q;

    PLL_pid.Kp = 5;
    PLL_pid.Ki = 1;

    PLL_pid.upper_limit = +PIEx100;
    PLL_pid.lower_limit = -PIEx100;

    Pid_calculation(&PLL_pid);

    PLL_theta = PIEx100 - PLL_pid.uo;
    G_theta.theta = G_theta.theta + PLL_theta * delta_time;

    G_theta.theta = G_theta.theta > PIEx2 ? (G_theta.theta - PIEx2) : G_theta.theta;
    G_theta.theta = G_theta.theta < 0 ? (G_theta.theta + PIEx2) : G_theta.theta;

    // test5 = 60 * G_theta.theta;
    // test6 = Vol_Vs.a;
    // test7 = GPark.q;
}

void VSG_Params_INIT(VSG_Params *p)
{
    p->J = 0.05; // 惯量
    p->D = 3000; // 阻尼

    p->Kw = 1; // 一次调频系数
    p->Kv = 0; // 一次调压系数

    p->P_ref = 9000; // 参考有功功率
    p->Q_ref = 0;    // 参考无功功率

    p->w0 = PIEx100;   // 额定角频率
    p->Vrms_nom = 220; // 额定相电压有效值

    p->System_w = PIEx100; // vsg输出角频率
    p->System_f = 50;      // vsg输出频率
    p->System_V = 311;     // vsg输出相电压幅值
    p->Sample_RMS = 0;     // 采样相电压有效值
}

void VSG_UPDATE(VSG_Params *vsg_params)
{

    /*
    // float one_w = (vsg_params->w0 - vsg_params->System_w) * vsg_params->Kw;
    vsg_params->alpha = ((vsg_params->P_ref - Sample_Pe) - vsg_params->D * (vsg_params->System_w - vsg_params->w0)) / vsg_params->J / vsg_params->w0;
    vsg_params->w_w0 += vsg_params->alpha * delta_time;

    // test4 = vsg_params->P_ref - Sample_Pe;

    vsg_params->System_w = vsg_params->w_w0 + vsg_params->w0;

    // test3 = vsg_params->System_w;

    vsg_params->System_f = vsg_params->System_w / PIEx2;

    VSG_theta.theta = VSG_theta.theta + vsg_params->System_w * delta_time;
    */
    vsg_params->System_w1 = delta_time * (vsg_params->P_ref - Sample_Pe) / vsg_params->J / vsg_params->w0 + (1 - delta_time * (vsg_params->D) / vsg_params->J / 314.15927) * vsg_params->System_w + delta_time / vsg_params->J * vsg_params->D; //
    vsg_params->System_w = vsg_params->System_w1;
    VSG_theta.theta = VSG_theta.theta + delta_time * vsg_params->System_w;

    VSG_theta.theta = VSG_theta.theta > PIEx2 ? (VSG_theta.theta - PIEx2) : VSG_theta.theta;
    VSG_theta.theta = VSG_theta.theta < 0 ? (VSG_theta.theta + PIEx2) : VSG_theta.theta;

    vsg_params->System_f = vsg_params->System_w / PIEx2;

    test1 = Sample_Pe;
    test2 = vsg_params->System_w;
    test3 = 50 * VSG_theta.theta;
    test4 = vsg_params->System_f;

    // test7 = 9000;
    // test8 = Sample_Pe;

    /***************************************************************************/

    VSG_pid.ref = vsg_params->Q_ref;
    VSG_pid.fdb = Sample_Qe;

    VSG_pid.Kp = 0;
    VSG_pid.Ki = 1;

    Id_pid.upper_limit = 400;  // d轴PID限幅
    Id_pid.lower_limit = -400; // d轴PID限幅

    Pid_calculation(&VSG_pid); // 计算出q轴pid输出

    vsg_params->System_V = 311 + VSG_pid.uo; // 机端电压幅值

    // test4 = VSG_pid.err;
}
