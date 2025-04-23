#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "main.h"

#define switch_loop 0 // 0：双闭环  1：单闭环

// 逆变器参数
#define Ub 6000.0            // 输出线电压幅值
#define Sb 800000.0          // 额定容量800kVA
#define switch_freq 16000.0  // 10k
#define delta_time 0.0000625 // 1/switch_freq
#define inv_params_L1 0.9e-3
#define inv_params_L2 0.1e-3
#define inv_params_C 40e-6
#define inv_params_RL 0.01
#define inv_params_RC 0.1

// 有源阻尼参数
#define damping 0.01 // 有源阻尼系数

// 原PID参数
#define U_Kp_parameter 1   // 1
#define U_Ki_parameter 500 // 500

#define I_Kp_parameter 0.006 // 0.006
#define I_Ki_parameter 5     //        5

/******************跟网并网*******************/
/*****current_Iabc******/
// #define I_Kp_parameter 0.006
// #define I_Ki_parameter 5

/******************离网*******************/
// #define U_Kp_parameter 0.06 // 0.6
// #define U_Ki_parameter 100  // 1500

// #define I_Kp_parameter 0.01 // 0.01
// #define I_Ki_parameter 10   // 10

#endif
