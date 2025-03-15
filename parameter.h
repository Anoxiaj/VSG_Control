#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "main.h"

#define switch_loop 0 // 0：双闭环  1：单闭环

// 逆变器参数
#define Ub 6000.0           // 输出线电压幅值
#define Sb 800000.0         // 额定容量800kVA
#define switch_freq 20000.0 // 20k
#define delta_time 0.00005  // 1/switch_freq
#define inv_params_L1 3e-3
#define inv_params_L2 1e-3
#define inv_params_C 35e-6
#define inv_params_RL 0.05
#define inv_params_RC 0.5

// VSG参数

// #define MAX_VOLTAGE 400
// #define MIN_VOLTAGE 360

// #define VSG_Kp_parameter 0
// #define VSG_Ki_parameter 0.5 * sqrt(3) * Sb / Ub

// 原PID参数
#define U_Kp_parameter 5
#define U_Ki_parameter 2000

#define I_Kp_parameter 0.2 // 0.004
#define I_Ki_parameter 200 // 3

/******************离网直流侧350V时*******************/
// f = 10e3;
// Ts = 1 / f;
// Vdc = 350;
// L = 2e-3;
// C = 40e-6;
// R = 1;
// 电流环参数：Kp = 0.05 Ki = 5
// 电压环参数如下：
// 单闭环时：Kp = 0.05 Ki = 5
// 双闭环时：100: 2 300 ; 150:0.1 50

#endif
