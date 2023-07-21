//
// Created by 神奇bug在哪里 on 7/15/23.
//

#ifndef SEARCHCAR_PID_H
#define SEARCHCAR_PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_last;
    float integral;
    float derivative;
    float result;
} PID;
void PID_init(PID *pid);
float PID_realize(PID *pid, float target, float actual);

#endif //SEARCHCAR_PID_H
