/**
 * @file pid.c
 * @author ZheWana
 * @brief PID控制器源文件
 * @version 0.2
 * @date 2023-08-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "./pid.h"
#include "main.h"
#include "math.h"

////////////////////////////////    Position PID    ///////////////////////////////////////////

psiFloat_t PID_RealizePsi(psiPid_t* pid, psiFloat_t input)
{
    pid->ctr.cur = input;

    pid->error.cur = pid->ctr.aim - pid->ctr.cur;
    pid->error.sum += pid->error.cur;
    pid->error.bia = pid->error.cur - pid->error.pre;
    pid->error.pre = pid->error.cur;

    pid->ctr.pre = pid->ctr.cur;
    return pid->kp * pid->error.cur + pid->ki * pid->error.sum + pid->kd * pid->error.bia;
}

psiFloat_t PID_RealizeForAnglePsi(psiPid_t* pid, psiFloat_t input)
{
    pid->ctr.cur = input;

    pid->error.cur = pid->ctr.aim - pid->ctr.cur;
    if (fabsf(pid->error.cur) > M_PI && fabsf(pid->error.cur) < 2 * M_PI) { // 角度溢出处理
        if (pid->error.cur > 0) {
            pid->error.cur -= 2 * M_PI;
        } else if (pid->error.cur < 0) {
            pid->error.cur += 2 * M_PI;
        }
    }
    pid->error.sum += pid->error.cur;
    pid->error.bia = pid->error.cur - pid->error.pre;
    pid->error.pre = pid->error.cur;

    pid->ctr.pre = pid->ctr.cur;
    return pid->kp * pid->error.cur + pid->ki * pid->error.sum + pid->kd * pid->error.bia;
}

// PID初始化函数
void PID_InitPsi(psiPid_t* pid, psiFloat_t kp, psiFloat_t ki, psiFloat_t kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_ResetPsi(psiPid_t* pid)
{
    pid->ctr.cur = 0;
    pid->ctr.pre = 0;
    pid->error.cur = 0;
    pid->error.pre = 0;
    pid->error.bia = 0;
    pid->error.sum = 0;
}

void PID_SetAimPsi(psiPid_t* pid, psiFloat_t aim)
{
    pid->ctr.aim = aim;
}

////////////////////////////////    Increment PID    ///////////////////////////////////////////

void PID_InitInc(incPID_t* pid, incFloat_t kp, incFloat_t ki, incFloat_t kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetAimInc(incPID_t* pid, incFloat_t aim)
{
    pid->ctr.aim = aim;
}

void PID_ResetInc(incPID_t* pid)
{

    pid->ctr.cur = 0;
    pid->ctr.pre = 0;
    pid->e[0] = pid->e[1] = pid->e[2] = 0;
}

incFloat_t PID_RealizeInc(incPID_t* pid, incFloat_t input)
{
    pid->ctr.cur = input;

    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = pid->ctr.aim - pid->ctr.cur;

    pid->ctr.pre = pid->ctr.cur;

    return pid->kp * (pid->e[0] - pid->e[1]) + pid->ki * pid->e[0]
        + pid->kd * (pid->e[0] - 2 * pid->e[1] + pid->e[2])
}