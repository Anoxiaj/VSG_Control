#include "main.h"

int jishu = 0;
float theta_50Hz, PLL_theta;
#define PIE 3.1415926535897932384626433832795
const float PIEx2 = 6.283185307179586476925286766559;
const float PIEx100 = 314.15926535897932384626433832795;

float waveA, waveB, waveC;

float test1, test2, test3;

float back_d, back_q; // 双闭环传递参数

float alpha;

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
VSG_Params vsg_params;

// for pid control
float U_feedback_d, U_feedback_q;
float I_feedback_d, I_feedback_q;

/// @brief Generate the angle of system control
void THETA_GENERATE(void)
{
    /* theta=wt , w=2pie f */
    theta_50Hz = theta_50Hz + delta_time * PIEx100; // 0.0001:interrupt time 10K
    theta_50Hz = theta_50Hz > PIEx2 ? (theta_50Hz - PIEx2) : theta_50Hz;
    theta_50Hz = theta_50Hz < 0 ? (theta_50Hz + PIEx2) : theta_50Hz;

    U_theta.theta = theta_50Hz;
    I_theta.theta = theta_50Hz;

    // test1 = U_theta.theta;
    // test2 = PIEx2;
}

void OPEN_LOOP(float Modulation)
{
    DQ2ABC v;
    v.d = Modulation;
    v.q = 0;
    dq2abc(&v, &I_theta);

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
    Ud_pid.upper_limit = 500;  // d轴PID限幅
    Ud_pid.lower_limit = -500; // d轴PID限幅
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

    Uq_pid.upper_limit = 1;  // d轴PID限幅
    Uq_pid.lower_limit = -1; // d轴PID限幅

    Pid_calculation(&Uq_pid); // 计算出q轴pid输出

    /*把pid计算出的dq输出值赋给park反变换的dq*/
    UiPark.d = Ud_pid.uo;
    UiPark.q = Uq_pid.uo;

    iPark(&UiPark, &VSG_theta); // Park反变换

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
    // test1 = Ud_pid.uo;
    // test2 = Uq_pid.uo;
    // test3 = Ud_pid.uo;
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

    iPark(&IiPark, &VSG_theta); // Park反变换

    /*把Park反变换的α和β赋给Clark反变换的α和β*/
    IiClark.alpha = IiPark.alpha;
    IiClark.beta = IiPark.beta;

    iClark(&IiClark); // Clark反变换

    /*有源阻尼测试代码*/
    // if (jishu > 1000)
    // {
    //     IiClark.a = IiClark.a - Sample_curr_Ca * 0.01;
    //     IiClark.b = IiClark.b - Sample_curr_Cb * 0.01;
    //     IiClark.c = IiClark.c - Sample_curr_Cc * 0.01;
    // }

    /*对调制波进行归一化（SPWM的相电压最大值为(Vdc/2)*/
    waveA = (IiClark.a + 1) / 2;
    waveB = (IiClark.b + 1) / 2;
    waveC = (IiClark.c + 1) / 2;

    jishu++;
    // test1 = Curr_Iabc.d;
    // test2 = Curr_Iabc.q;
    // test3 = Id_pid.uo;
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

    // test1 = 60 * G_theta.theta;
    // test2 = Vol_Vs.a;
    // test3 = GPark.q;
}

/// @brief Virtual Synchronous Generator (VSG) control function
/// This function initializes VSG parameters, calculates w and voltage deviations,
/// computes power outputs, updates system w and voltage, and ensures they are within limits.
/// @param VSG_theta.theta
/// @param vsg_params.Em
void VSG_Control(VSG_Params *p)
{
    // 初始化VSG参数：D↑阻尼越大 J(0.005~0.3)↑ 极点右移增加不稳定风险（不宜过大）
    p->J = 0.02;                // 虚拟惯量系数
    p->D = 50;                  // 阻尼系数
    p->Kp_f = 0.95e4 / PIEx100; // 频率-有功下垂系数
    p->Kq_v = 0.1;              // 电压-无功下垂系数
    p->wnom = PIEx100;          // 额定角频率：对应50Hz
    p->Vnom = 311.0;            // 额定电压峰幅
    p->P_ref = 14.6e3;          // 有功功率给定
    p->Q_ref = 0;               // 无功功率给定

    // 计算角频率偏差（w0-w)
    float w_deviation = p->System_w - p->wnom; // System_w为系统角频率

    // 计算Tm-Te-D(w-w0)输出
    alpha = ((p->P_ref - Sample_Pe) / p->wnom - p->D * w_deviation) / p->J; // P_ref为有功功率给定值，Sample_Pe为电磁功率

    // 更新系统角频率w
    p->System_w += alpha * delta_time; //
    // p->System_w = p->System_w + p->wnom;

    // 更新系统频率
    p->System_f = p->System_w / PIEx2;

    // 更新系统相角wt
    VSG_theta.theta += p->System_w * delta_time;

    // θ角mod函数-->确保θ在0-2pi变换
    VSG_theta.theta = VSG_theta.theta > PIEx2 ? (VSG_theta.theta - PIEx2) : VSG_theta.theta;
    VSG_theta.theta = VSG_theta.theta < 0 ? (VSG_theta.theta + PIEx2) : VSG_theta.theta;

    /*******************************************************/
    /// @brief*无功电压控制
    /*******************************************************/

    // 计算电压偏差
    float voltage_deviation = p->Vnom - p->System_V; // System_Voltage为系统电压

    // 计算无功功率输出
    float Q_output = p->Q_ref + voltage_deviation * p->Kq_v; // Q_ref为无功功率给定值

    VSG_pid.ref = Q_output;  // PID无功功率给定值
    VSG_pid.fdb = Sample_Qe; // 采样无功功率归一化

    VSG_pid.Kp = 0;
    VSG_pid.Ki = 100;

    VSG_pid.upper_limit = 400;  // d轴PID限幅
    VSG_pid.lower_limit = -400; // d轴PID限幅

    // 更新系统电压
    Pid_calculation(&VSG_pid);

    p->System_V = VSG_pid.uo + p->Vnom; // 311为相电压的峰值
    // p->System_V = 311;

    // // 给输出限幅
    // if (vsg_params->Em >= MAX_VOLTAGE)
    // {
    //     vsg_params->Em = MAX_VOLTAGE;
    // }
    // else if (vsg_params->Em <= MIN_VOLTAGE)
    // {
    //     vsg_params->Em = MIN_VOLTAGE;
    // }
    // else
    // {
    //     vsg_params->Em = vsg_params->Em;
    // }

    test1 = Sample_Pe;
    // test2 = p->System_f;
    // // test1 = VSG_pid.err;
    test2 = Sample_Qe;
    test3 = p->System_V;
}
