#include "pid.h"
#include "dwt.h"
#include "bsp_timer.h"

//梯形积分,把积分项从矩形积分改成梯形积分，使用当前误差和上一次误差平均值
//适用场景:周期较长的pid计算,可以提高积分精度,减少积分误差
static void Pid_T_Intergral(PID_Instance_s *pid)
{
    //梯形积分
    pid->iterm = pid->ki * (pid->err[0] + pid->err[1]) * pid->dt / 2.0f;
}

//变速积分,误差小于core_b时积分正常,误差大于core_a时积分为0,误差介于core_a和core_b之间时积分逐渐变大
//适用场景:误差较大时增大积分作用,误差较小时减小积分作用
static void Pid_Change_I(PID_Instance_s *pid)
{
    if (pid->err[0] < pid->Improve_param.core_b)
        return;
    if (pid->err[0] < pid->Improve_param.core_a + pid->Improve_param.core_b)
        pid->iterm *= (pid->Improve_param.core_a + pid->Improve_param.core_b - pid->err[0]);
    else
        pid->iterm *= 0;
}

//积分限幅
static void Pid_IntegralLimit(PID_Instance_s *pid)
{
    static float temp_output, temp_iout;
    temp_iout = pid->iout + pid->iterm;
    temp_output = pid->pout + temp_iout + pid->dout;
    if (fabsf(temp_output) > pid->maxout){
        if (pid->err[0] * pid->iterm > 0) {
            pid->iterm = 0;
        }
    }

    if (temp_iout > pid->Improve_param.i_limit) {
        pid->iterm = 0;
        pid->iout = pid->Improve_param.i_limit;
    }
    if (temp_iout < -pid->Improve_param.i_limit) {
        pid->iterm = 0;
        pid->iout = -pid->Improve_param.i_limit;
    }
}

//微分先行,避免目标致突变导致微分项剧烈变化
static void Pid_Derivative_On_Measurement(PID_Instance_s *pid)
{
    pid->dout = pid->kd * (pid->last_measure - pid->measure) / pid->dt;
}

//微分滤波
static void Pid_Derivative_Filter(PID_Instance_s *pid)
{
    pid->dout = pid->dout * pid->dt / (pid->Improve_param.derivative_LPF_RC + pid->dt) + 
                pid->last_dout * pid->Improve_param.derivative_LPF_RC / (pid->Improve_param.derivative_LPF_RC + pid->dt);
}

//输出滤波
static void Pid_Output_Filter(PID_Instance_s *pid)
{
    pid->output = pid->output * pid->dt / (pid->Improve_param.output_LPF_RC + pid->dt) +
                  pid->output * pid->Improve_param.output_LPF_RC / (pid->Improve_param.output_LPF_RC + pid->dt);
}

//变速P,误差越大,P越大
static void Pid_Change_P(PID_Instance_s *pid)
{
    float abs_err = fabsf(pid->err[0]);
    float kp_dynamic = pid->kp;

    // 参数非法时回退到固定kp
    if ((pid->Improve_param.err_max > 0.0f) &&
        (pid->Improve_param.p_max > pid->Improve_param.p_min)) {
        float ratio = abs_err / pid->Improve_param.err_max;

        if (ratio > 1.0f) {
            ratio = 1.0f;
        }

        kp_dynamic = pid->Improve_param.p_min +
                     (pid->Improve_param.p_max - pid->Improve_param.p_min) * ratio;
    }

    pid->pout = kp_dynamic * pid->err[0];

}

//输出限幅
static void OutputLimit(PID_Instance_s *pid)
{
    if (pid->output > pid->maxout) {
        pid->output = pid->maxout;
    } else if (pid->output < -pid->maxout) {
        pid->output = -pid->maxout;
    }
}

float PIDCalculate(PID_Instance_s *pid, float measure, float target)
{
    //获取两次计算的时间间隔
    pid->dt = DWT_GetDeltaT_s(&pid->DWT_CNT);

    pid->measure = measure;
    pid->err[1] = pid->err[0];
    pid->err[0] = target - measure;
    if (fabsf(pid->err[0]) > pid->deadband) {
        // 基本的pid计算,使用位置式-时间积分
        
        pid->pout = pid->kp * pid->err[0];
        pid->iterm = pid->ki * pid->err[0] * pid->dt;
        pid->dout = pid->kd * (pid->err[0] - pid->err[1]) / pid->dt;

        if (pid->Improve & PID_Changing_P) {
            Pid_Change_P(pid);
        }

        if (pid->Improve & PID_T_Intergral) {
            Pid_T_Intergral(pid);
        }

        if (pid->Improve & PID_Changing_I) {
            Pid_Change_I(pid);
        }

        if (pid->Improve & PID_I_limit) {
            Pid_IntegralLimit(pid);
        }

        if (pid->Improve & PID_D_On_Measurement) {
            Pid_Derivative_On_Measurement(pid);
        }

        if (pid->Improve & PID_D_Filter) {
            Pid_Derivative_Filter(pid);
        }

        pid->iout += pid->iterm;
        pid->output = pid->pout + pid->iout + pid->dout;

        if (pid->Improve & PID_OutputFilter) {
            Pid_Output_Filter(pid);
        }

        OutputLimit(pid);
    } else {
        pid->pout = 0;
        pid->iterm = 0;
    }

    pid->last_measure = pid->measure;
    pid->last_dout = pid->dout;
    pid->last_iterm = pid->iterm;

    return pid->output;
}