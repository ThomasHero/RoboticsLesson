//
// Created by thomas on 20-3-11.
//

#ifndef SIMULATION_WORKSPACE_PID_CORE_H
#define SIMULATION_WORKSPACE_PID_CORE_H

typedef struct {
    float kp;
    float ki;
    float kd_fdbk;

    float err_lower_limit;
    float err_uper_limit;
    float I_lower_limit;
    float I_uper_limit;
    float D_lower_limit;
    float D_uper_limit;

    float output_lower_limit;
    float output_uper_limit;

    float err;
    float err_old;
    float diff_fdbk;
    float P;
    float I;
    float D_fdbk;
    float PID_out;

} PID_CORE_T;
extern void pid_init(PID_CORE_T *pid);
extern void pid_reset(PID_CORE_T *pid);
extern float pid_core_exe(PID_CORE_T * PID_core,float expectVal ,float curVal,float dltTime);
extern float pid_inc_core_exe(PID_CORE_T * PID_core,float cmd ,float fdbk ,float dltT);
extern void pid_set_param_pid(PID_CORE_T * PID_core,float p,float i,float d);
extern void pid_set_param_limit(PID_CORE_T * PID_core,
                                float errlow,    float errupper, float i_low  ,   float i_upper,
                                float outlow,    float outupper, float d_lower,   float d_upper );
extern void pid_set_param_limit_err(PID_CORE_T * PID_core,float errlow,float errupper);
extern void pid_set_param_limit_out(PID_CORE_T * PID_core,float outlow,float outupper);
extern void pid_set_param_limit_D(PID_CORE_T * PID_core,float outlow,float outupper);
extern void pid_set_param_limit_I(PID_CORE_T * PID_core,float outlow,float outupper);
extern float constrain(float val, float min, float max);

#endif //SIMULATION_WORKSPACE_PID_CORE_H
