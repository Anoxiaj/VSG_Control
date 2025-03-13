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

// VSG参数结构体
typedef struct
{
    float J;        // 虚拟惯量系数 (s)
    float D;        // 阻尼系数
    float Kp_f;     // 频率-有功下垂系数 (Hz/kW)
    float Kq_v;     // 电压-无功下垂系数 (V/kVar)
    float wnom;     // 额定角频率 (rad/s)
    float Vnom;     // 额定电压幅值 (V)
    float P_ref;    // 有功功率给定 (W)
    float Q_ref;    // 无功功率给定 (Var)
    float System_w; // 系统角频率 (rad/s)
    float System_V; // 系统电压幅值 (V)
    float System_f; // 系统频率 (Hz)
    float Em;       // 输出相电压幅值
} VSG_Params;

extern int jishu;

extern float waveA, waveB, waveC;
extern float theta_50Hz, PLL_theta;

extern float back_d, back_q;

extern float test1, test2, test3;

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
extern VSG_Params vsg_params; // 全局VSG参数实例

extern float U_feedback_d, U_feedback_q;
extern float I_feedback_d, I_feedback_q;

extern void THETA_GENERATE(void);                                                            // Generate the angle of system control for 50Hz(离网)
extern void OPEN_LOOP(float Modulation);                                                     // Open loop control
extern void VOLTAGE_CLOSED_LOOP(float V_ref);                                                // Voltage closed loop control
extern void CURRENT_CLOSED_LOOP(float I_ref, float I_q, float d_feedback, float q_feedback); // Current closed loop control
extern void PHASE_LOCKED_LOOP(void);// Phase-Locked Loop (PLL) control function
extern void VSG_Control(VSG_Params *p);                                                 // VSG control function

#endif /* APP_INVERTER_H_ */