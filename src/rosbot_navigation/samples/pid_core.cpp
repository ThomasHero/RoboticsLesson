//
// Created by thomas on 20-3-11.
//

#include "pid_core.h"
#include "stdio.h"

#define DBG_I(fmt,args...)
//printf(fmt,##args)

#define DEFAULT_VALUE 100
void pid_init(PID_CORE_T *pid)
{
    pid->D_fdbk             = 0;
    pid->I                  = 0;
    pid->P                  = 0;
    pid->kp                 = 0;
    pid->ki                 = 0;
    pid->kd_fdbk            = 0;
    pid->err                = 0;
    pid->err_old            = 0;
    pid->PID_out            = 0;
    pid->err_uper_limit     =  DEFAULT_VALUE;
    pid->err_lower_limit    = -DEFAULT_VALUE;
    pid->output_uper_limit  =  5;
    pid->output_lower_limit = -5;
    pid->D_uper_limit       =  20;
    pid->D_lower_limit      = -20;
    pid->I_uper_limit       =  10;
    pid->I_lower_limit      = -10;
}
void pid_reset(PID_CORE_T *pid)
{
    pid->D_fdbk             = 0;
    pid->I                  = 0;
    pid->P                  = 0;
    pid->err                = 0;
    pid->err_old            = 0;
    pid->PID_out            = 0;
}

void pid_set_param_pid(PID_CORE_T * PID_core,float kp,float ki,float kd)
{
    PID_core->kp = kp;
    PID_core->ki = ki;
    PID_core->kd_fdbk = kd;
}

void pid_set_param_limit(PID_CORE_T * PID_core,
                         float errlow,    float errupper,
                         float ilow  ,    float iupper,
                         float outlow,    float outupper,
                         float d_lower,   float d_upper )
{
    PID_core->err_uper_limit    = errupper;
    PID_core->err_lower_limit   = errlow;
    PID_core->output_uper_limit = outupper;
    PID_core->output_lower_limit= outlow;
    PID_core->I_uper_limit      = iupper;
    PID_core->I_lower_limit     = ilow;
}

void pid_set_param_limit_err(PID_CORE_T * PID_core,float errlow,float errupper)
{
    PID_core->err_uper_limit    = errupper;
    PID_core->err_lower_limit   = errlow;
}
void pid_set_param_limit_out(PID_CORE_T * PID_core,float outlow,float outupper)
{
    PID_core->output_uper_limit = outupper;
    PID_core->output_lower_limit= outlow;
}

void pid_set_param_limit_D(PID_CORE_T * PID_core,float outlow,float outupper)
{
    PID_core->D_uper_limit=outupper;
    PID_core->D_lower_limit=  outlow;
}
void pid_set_param_limit_I(PID_CORE_T * PID_core,float outlow,float outupper)
{
    PID_core->I_uper_limit= outupper;
    PID_core->I_lower_limit= outlow;
}
float f_err,f_p,f_i,f_d,f_out;
float pid_core_exe(PID_CORE_T * PID_core,float cmd ,float fdbk ,float dltT)
{
    PID_core->err 		= cmd - fdbk;
    PID_core->err		= constrain(PID_core->err,PID_core->err_lower_limit,PID_core->err_uper_limit);
    f_err = PID_core->err;
    PID_core->diff_fdbk = (PID_core->err - PID_core->err_old)/dltT;

    PID_core->P		    = PID_core->kp*PID_core->err;
    f_p = PID_core->P;
    PID_core->I		   += PID_core->ki*(PID_core->err*dltT);
    PID_core->I         = constrain(PID_core->I, PID_core->I_lower_limit, PID_core->I_uper_limit);
    f_i = PID_core->I;
    PID_core->D_fdbk    = PID_core->kd_fdbk*PID_core->diff_fdbk;
    PID_core->D_fdbk    = constrain(PID_core->D_fdbk,PID_core->D_lower_limit,PID_core->D_uper_limit);
    f_d = PID_core->D_fdbk;
    PID_core->PID_out   = PID_core->P + PID_core->I + PID_core->D_fdbk;
    PID_core->PID_out   = constrain(PID_core->PID_out,PID_core->output_lower_limit,PID_core->output_uper_limit);
    f_out = PID_core->PID_out;
    DBG_I("ERR     %f",PID_core->err);
    DBG_I("P       %f",PID_core->P);
    DBG_I("D_fdbk  %f",PID_core->D_fdbk);
    DBG_I("PID_out %f \r\n",PID_core->PID_out);
    PID_core->err_old	= PID_core->err;
    return PID_core->PID_out;
}
float pid_inc_core_exe(PID_CORE_T * PID_core,float cmd ,float fdbk ,float dltT)
{
    float cur_diff;
    PID_core->err 		= cmd - fdbk;
    PID_core->err		= constrain(PID_core->err,PID_core->err_lower_limit,PID_core->err_uper_limit);
    f_err = PID_core->err;
    cur_diff            = (PID_core->err-PID_core->err_old)/dltT;

    PID_core->P		    = PID_core->kp*cur_diff;
    f_p = PID_core->P;
    PID_core->I		    = PID_core->ki*PID_core->err;
    PID_core->I         = constrain(PID_core->I, PID_core->I_lower_limit, PID_core->I_uper_limit);
    f_i = PID_core->I;
    PID_core->D_fdbk    = PID_core->kd_fdbk * (cur_diff - PID_core->diff_fdbk);
    PID_core->D_fdbk    = constrain(PID_core->D_fdbk, PID_core->D_lower_limit, PID_core->D_uper_limit);
    f_d = PID_core->D_fdbk;
    PID_core->PID_out  += (PID_core->P + PID_core->I + PID_core->D_fdbk);
    PID_core->PID_out   = constrain(PID_core->PID_out,PID_core->output_lower_limit,PID_core->output_uper_limit);
    f_out = PID_core->PID_out;
    DBG_I("ERR     %f",PID_core->err);
    DBG_I("P       %f",PID_core->P);
    DBG_I("D_fdbk  %f",PID_core->D_fdbk);
    DBG_I("PID_out %f \r\n",PID_core->PID_out);
    PID_core->err_old	= PID_core->err;
    PID_core->diff_fdbk = cur_diff;
    return PID_core->PID_out;
}

float constrain(float val, float min, float max)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}
