#include "mex.h"  // MATLAB接口头文件
#include "main.h" // 主控制头文件（已包含parameter.h）

static int pulse_f = 0;
static int pulse_f_Old = 0; // 上面两个参数控制代码每周期只执行一次 模拟芯片里的操作
float32 m = 0;				// 调制度

/*变量定义*/

void VSG_control_main(double out_var[9], double in_var[15]) // 相当于主函数名：example_func【可以按照想法更改，最后一行处也要改】// out_var[6]输出变量，个数为6  in_var[6]输出变量，个数为6
{
	/*
	in_var[0]:更新信号
	in_var[1]:初始化信号
	in_var[2]:载波信号
	in_var[3-5]:机端电压信号
	in_var[6-8]:逆变器输出电流信号（桥臂侧）
	in_var[9-11]:机端电流信号
	in_var[12]:电磁有功功率
	in_var[13]:电磁无功功率
	*/

	pulse_f = in_var[0];

	/*given*/
	// Vref = in_var[0];	// 参考相电压
	Iref = 311; // 参考电感电流
	// Vdc = in_var[8];	// 直流母线电压
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
		vsg_params.System_w = 314.15926535897932384626433832795;
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

		Sample_Pe = in_var[12]; // 电磁功率采样变量
		Sample_Qe = in_var[13];

		vsg_params.System_V = in_var[14]; // 系统电压有效值

		/******************************************/

		// PHASE_LOCKED_LOOP(); // 角度生成-->G_theta

		// VSG_Control(&vsg_params); // VSG控制-->VSG_theta,Em

		THETA_GENERATE();	  // 角度生成-->U_theta, I_theta
		INV_XY_CAL(&I_theta); // 采样信号的坐标变换-->(Vol_Vs, Curr_Iabc, Curr_Is)的 d,q

		// OPEN_LOOP(m);
		// VOLTAGE_CLOSED_LOOP(vsg_params.Em);

#if switch_loop
		CURRENT_CLOSED_LOOP(Iref, 0, Curr_Iabc.d, Curr_Iabc.q); // 电流单闭环
#else
		CURRENT_CLOSED_LOOP(back_d, back_q); // 双闭环
#endif
	}

	// 4、载波调制 载波in_var[5]; 因为脉冲要一直比较，所以放到最外层，每个仿真时间执行一次
	// 无死区；
	out_var[0] = waveA > in_var[2] ? 1 : 0;
	out_var[1] = 1 - out_var[0];
	out_var[2] = waveB > in_var[2] ? 1 : 0;
	out_var[3] = 1 - out_var[2];
	out_var[4] = waveC > in_var[2] ? 1 : 0;
	out_var[5] = 1 - out_var[4];

	// out_var[0] = Vol_Vs.d;
	// out_var[1] = Vol_Vs.q;
	// out_var[2] = Curr_Iabc.d;
	// out_var[3] = Curr_Iabc.q;
	// out_var[4] = Curr_Is.d;
	// out_var[5] = Curr_Is.q;
	// 测试端口6-8
	out_var[6] = test1;
	out_var[7] = test2;
	out_var[8] = test3;

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
	plhs[0] = mxCreateDoubleMatrix(9, 1, mxREAL); // 输出矩阵 列向量 这里输出数据的个数要和模块里的个数一一对应
	//(6,1,mxREAL)  【参数6表示输出变量维数（输出变量个数）】  【1和mxREAL默认不变】

	out = mxGetPr(plhs[0]);
	in = mxGetPr(prhs[0]);
	VSG_control_main(out, in); // example_func为主函数名【可以按照想法更改】    out->plhs     in->prhs
}
