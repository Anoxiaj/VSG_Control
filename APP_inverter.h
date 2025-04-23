#ifndef APP_INVERTER_H_
#define APP_INVERTER_H_

#include "main.h"

// DQ->ABC
typedef struct
{
    float d;
    float q;
    float a;
    float b;
    float c;
} DQ2ABC;
#define DQ2ABC_DEFAULTS {0, 0, 0, 0, 0}

// ABC->DQ
typedef struct
{
    float a;
    float b;
    float c;
    float d;
    float q;
} ABC2DQ;
#define DQ2ABC_DEFAULTS {0, 0, 0, 0, 0}

// park变换
typedef struct
{
    float alpha;
    float beta;
    float d;
    float q;
} PARK_REGS;
#define PARK_REGS_DEFAULTS {0, 0, 0, 0}

// park反变换
typedef struct
{
    float alpha;
    float beta;
    float d;
    float q;
} IPARK_REGS;
#define IPARK_REGS_DEFAULTS {0, 0, 0, 0}

// clark变换
typedef struct
{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
} CLARK_REGS;
#define CLARK_REGS_DEFAULTS {0, 0, 0, 0, 0}

// clark反变换
typedef struct
{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
} ICLARK_REGS;
#define ICLARK_REGS_DEFAULTS {0, 0, 0, 0, 0}

/*斜坡给定*/
typedef struct
{
    float Given;
    float output;
    float delta;
    unsigned int count;
    unsigned int length;
} RAMP_REFERENCE;

/*调节器结构体*/
typedef struct
{
    float Kp;
    float Ki;
    float Kc;
    float ref;
    float fdb;
    float err;
    float ui;
    float up;
    float upresat;
    float uo;
    float upper_limit;
    float lower_limit;
} PID;
#define PID_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

typedef struct
{
    double J;  // 惯量
    double D;  // 阻尼
    double Kw; // 一次调频系数
    double Kv; // 一次调压系数

    double w0;       // 额定角频率
    double Vrms_nom; // 额定相电压有效值
    double P_ref;    // 参考有功功率
    double Q_ref;    // 参考无功功率

    double System_w;   // vsg k 时刻输出角频率
    double System_w1;  // vsg (k+1)时刻输出角频率
    double System_f;   // vsg输出频率
    double System_V;   // vsg输出相电压幅值
    double Sample_RMS; // 采样相电压有效值

    double alpha; // 角加速度
    double w_w0;  // 角速度差
} VSG_Params;

extern int jishu;

extern float waveA, waveB, waveC;
extern float theta_50Hz, PLL_theta;

extern float back_d, back_q;

extern float test1, test2, test3, test4;
extern float test5, test6, test7, test8;

extern PID Ud_pid;
extern PID Uq_pid;
extern PID Id_pid;
extern PID Iq_pid;
extern PID PLL_pid;
extern PID VSG_pid;

extern RAMP_REFERENCE Ud_ramp;
extern RAMP_REFERENCE Uq_ramp;
extern RAMP_REFERENCE Id_ramp;
extern RAMP_REFERENCE Iq_ramp;

VSG_Params vsg;

extern void THETA_GENERATE(void);                                                            // Generate the angle of system control for 50Hz(离网)
extern void OPEN_LOOP(float Modulation);                                                     // Open loop control
extern void VOLTAGE_CLOSED_LOOP(float V_ref, float V_q, float d_feedback, float q_feedback); // Voltage closed loop control
extern void CURRENT_CLOSED_LOOP(float I_ref, float I_q, float d_feedback, float q_feedback); // Current closed loop control
extern void PHASE_LOCKED_LOOP(void);                                                         // Phase-Locked Loop (PLL) control function
extern void VSG_Params_INIT(VSG_Params *p);                                                  // VSG参数初始化
extern void VSG_UPDATE(VSG_Params *vsg_params);                                              // VSG参数更新

#endif /* APP_INVERTER_H_ */