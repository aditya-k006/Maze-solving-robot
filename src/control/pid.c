#include "control/pid.h"
#include <math.h>

static float clamp(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
}

float pid_compute(pid_t *pid, float setpoint, float measured, float dt)
{
    float error      = setpoint - measured;
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;

    pid->integral  += error * dt;
    pid->integral   = clamp(pid->integral, pid->integral_limit);
    pid->prev_error = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    return clamp(output, pid->output_limit);
}

void pid_reset(pid_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
