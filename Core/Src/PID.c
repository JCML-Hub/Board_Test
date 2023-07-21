//
// Created by 神奇bug在哪里 on 7/15/23.
//

#include "PID.h"
void PID_init(PID *pid) {
    pid->error = 0;
    pid->error_last = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->result = 0;
}
float PID_realize(PID *pid, float target, float actual) {
    pid->error = target - actual;
    pid->integral += pid->error;
    pid->derivative = pid->error - pid->error_last;
    pid->error_last = pid->error;
    pid->result = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    return pid->result;
}