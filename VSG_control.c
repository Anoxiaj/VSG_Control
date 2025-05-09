#include "mex.h"  // MATLAB接口头文件
#include "main.h" // 主控制头文件（已包含parameter.h）

static int pulse_f = 0;
static int pulse_f_Old = 0; // 上面两个参数控制代码每周期只执行一次 模拟芯片里的操作

/*变量定义*/
float32 m = 0.5; // 调制度
// #define Iref 100 // 参考电流
float Iref = 100; // 参考电流
float Vref = 311; // 参考相电压
#define Vdc 850	  // 直流母线电压

void VSG_control_main(double out_var[14], double in_var[16]) // 相当于主函数名：example_func【可以按照想法更改，最后一行处也要改】// out_var[6]输出变量，个数为6  in_var[6]输出变量，个数为6
{
	/*
	in_var[0]:更新信号
	in_var[1]:初始化信号
	in_var[2]:载波信号
	in_var[3-5]:机端电压信号
	in_var[6-8]:逆变器输出电流信号（桥臂侧）
	in_var[9-11]:机端电流信号
	in_var[12]:输出电压有效值
	*/

	pulse_f = in_var[0];

	/*given*/

	// m = 2 * Vref / Vdc; // SPWM调制度
	m = 0.5;

	if (in_var[1] == 0)				   // 延时启动，一开始时钟信号都是0
	{								   // 初始化
		THETA_REGS_VAR_INIT(&U_theta); // 角度计算变量初始化
		THETA_REGS_VAR_INIT(&I_theta);
		THETA_REGS_VAR_INIT(&G_theta);
		THETA_REGS_VAR_INIT(&VSG_theta);
		PID_VAR_INIT(&Ud_pid); // PID参数变量初始化
		PID_VAR_INIT(&Uq_pid);
		PID_VAR_INIT(&PLL_pid);
		PID_VAR_INIT(&VSG_pid);

		RAMP_VAR_INIT(&Ud_ramp); // 斜坡给定变量初始化
		RAMP_VAR_INIT(&Uq_ramp);
		RAMP_VAR_INIT(&Id_ramp);
		RAMP_VAR_INIT(&Iq_ramp);

		VSG_Params_INIT(&vsg); // VSG参数变量初始化

		theta_50Hz = 0; // 角度生成变量初始化
		PLL_theta = 0;
		G_theta.theta = 0;
		I_theta.theta = 0;
		VSG_theta.theta = 0;
		waveA = 0; // 调制波
		waveB = 0;
		waveC = 0;
		m = 0;
		jishu = 0;

		// 滤波器初始化
		float dt = delta_time; // 采样周期（需与系统实际控制周期一致）
		float fc = 100.0;	   // 截止频率100Hz，决定滤除高频噪声的强度
		init_low_pass_filter(&filter_P, dt, fc);
		init_low_pass_filter(&filter_V, dt, fc);

		// theta_50Hz = -(3.1415926535897932384626433832795 / 2 + delta_time * 10 * 3.1415926535897932384626433832795); // 90°
	}

	if (pulse_f_Old == 0 && pulse_f == 1)
	{
		/***********Sample Allocation**************/
		Vol_Vs.a = in_var[3]; // 机端电压Vs采样变量
		Vol_Vs.b = in_var[4];
		Vol_Vs.c = in_var[5];

		Curr_Iabc.a = in_var[6]; // 逆变器输出电流Iabc采样变量
		Curr_Iabc.b = in_var[7];
		Curr_Iabc.c = in_var[8];

		Curr_Is.a = in_var[9]; // 机端电流Is采样变量
		Curr_Is.b = in_var[10];
		Curr_Is.c = in_var[11];

		Curr_Ic.a = in_var[13]; // 滤波电容电流采样变量
		Curr_Ic.b = in_var[14];
		Curr_Ic.c = in_var[15];

		update_low_pass_filter(&filter_V, in_var[12]);
		vsg.Sample_RMS = filter_V.y_prev; // 系统电压有效值

		PHASE_LOCKED_LOOP(); // 角度生成-->G_theta

		PQ_Calculation(&Vol_Vs, &Curr_Is); // PQ计算-->Sample_Pe，Sample_Qe

		VSG_UPDATE(&vsg); // VSG参数更新-->vsg.System_w, vsg.System_f, vsg.System_V

		THETA_GENERATE(); // 角度生成-->U_theta, I_theta

		INV_XY_CAL(&VSG_theta); // 采样信号的坐标变换-->(Vol_Vs, Curr_Iabc, Curr_Is)的 d,q

		// OPEN_LOOP(m);
		// VOLTAGE_CLOSED_LOOP(vsg.System_V, 0, Vol_Vs.d, Vol_Vs.q);
		VOLTAGE_CLOSED_LOOP(311, 0, Vol_Vs.d, Vol_Vs.q);

#if switch_loop
		CURRENT_CLOSED_LOOP(Iref, 0, Curr_Is.d, Curr_Is.q); // 电流单闭环
#else
		CURRENT_CLOSED_LOOP(back_d, back_q, Curr_Is.d, Curr_Is.q); // 双闭环
#endif
	}

	/************SVPWM调制*****************************************************/
	SVPWM_Control(&SVPWM_var);
	// 4、载波调制 载波in_var[2]; 因为脉冲要一直比较，所以放到最外层，每个仿真时间执行一次
	// 无死区；
	out_var[0] = vta < in_var[2] ? 1 : 0;
	out_var[1] = 1 - out_var[0];
	out_var[2] = vtb < in_var[2] ? 1 : 0;
	out_var[3] = 1 - out_var[2];
	out_var[4] = vtc < in_var[2] ? 1 : 0;
	out_var[5] = 1 - out_var[4];

	/************SPWM调制*****************************************************/
	// // 4、载波调制 载波in_var[5]; 因为脉冲要一直比较，所以放到最外层，每个仿真时间执行一次
	// // 无死区；
	// out_var[0] = waveA > in_var[2] ? 1 : 0;
	// out_var[1] = 1 - out_var[0];
	// out_var[2] = waveB > in_var[2] ? 1 : 0;
	// out_var[3] = 1 - out_var[2];
	// out_var[4] = waveC > in_var[2] ? 1 : 0;
	// out_var[5] = 1 - out_var[4];

	// out_var[0] = Vol_Vs.d;
	// out_var[1] = Vol_Vs.q;
	// out_var[2] = Curr_Iabc.d;
	// out_var[3] = Curr_Iabc.q;
	// out_var[4] = Curr_Is.d;
	// out_var[5] = Curr_Is.q;
	// 测试端口6-13
	out_var[6] = test1;
	out_var[7] = test2;
	out_var[8] = test3;
	out_var[9] = test4;

	out_var[10] = test5;
	out_var[11] = test6;
	out_var[12] = test7;
	out_var[13] = test8;

	// out_var[6] = Curr_Is.d;
	// out_var[7] = Curr_Is.q;

	// out_var[6] = waveA;
	// out_var[7] = waveB;
	// out_var[8] = waveC;

	pulse_f_Old = pulse_f;
}

void mexFunction( // 此处可更改地方
	int nlhs,	  //     |
	mxArray *plhs[],
	int nrhs, //     |
	const mxArray *prhs[])
{ //     |

	// if (nrhs != 1)
	// {
	// 	mexErrMsgIdAndTxt("VSG_control:invalidNumInputs", "One input vector (15 elements) is required.");
	// }
	// if (nlhs != 1)
	// {
	// 	mexErrMsgIdAndTxt("VSG_control:invalidNumOutputs", "One output vector (9 elements) is required.");
	// }

	float *out;
	float *in; //     |

	// allocate memory for output                           //     |
	plhs[0] = mxCreateDoubleMatrix(14, 1, mxREAL); // 输出矩阵 列向量 这里输出数据的个数要和模块里的个数一一对应
	//(6,1,mxREAL)  【参数6表示输出变量维数（输出变量个数）】  【1和mxREAL默认不变】

	out = mxGetPr(plhs[0]);
	in = mxGetPr(prhs[0]);
	VSG_control_main(out, in); // example_func为主函数名【可以按照想法更改】    out->plhs     in->prhs
}
