/**
 * @file pid.h
 * @author ZheWana
 * @brief PID控制器头文件
 * @version 0.2
 * @date 2023-08-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PID_H
#define PID_H

typedef float psiFloat_t;
typedef float incFloat_t;

#ifndef M_PI
#define M_PI 3.1415926535
#endif

////////////////////////////////    Positional PID    ///////////////////////////////////////////

typedef struct {
    psiFloat_t kp;
    psiFloat_t ki;
    psiFloat_t kd;

    struct ctr {
        psiFloat_t cur;
        psiFloat_t pre;
        psiFloat_t aim;
    } ctr;

    struct error {
        incFloat_t cur;
        incFloat_t pre;
        incFloat_t sum;
        incFloat_t bia;
    } error;

} psiPid_t;

/**
 * @brief 位置式PID初始化函数
 * @param pid pid句柄
 * @param kp 比例参数
 * @param ki 积分参数
 * @param kd 微分参数
 */
void PID_InitPsi(psiPid_t* pid, psiFloat_t kp, psiFloat_t ki, psiFloat_t kd);

/**
 * @brief 位置式PID设定目标值
 * @param pid pid句柄
 * @param aim 目标值
 */
void PID_SetAimPsi(psiPid_t* pid, psiFloat_t aim);

/**
 * @brief 位置式PID重置函数
 * @param pid pid句柄
 * @note 重置函数不对PID参数和目标值进行重置，仅对运算中间量进行重置
 */
void PID_ResetPsi(psiPid_t* pid);

/**
 * @brief 位置式PID实现函数
 * @param pid pid句柄
 * @param input 当前输入量
 * @return PID输出量
 */
float PID_RealizePsi(psiPid_t* pid, psiFloat_t input);

/**
 * @brief 带有角度溢出的位置式PID实现
 * @param pid pid句柄
 * @param input 当前输入量
 * @return PID输出量
 * @note 角度使用弧度制
 */
float PID_RealizeForAnglePsi(psiPid_t* pid, psiFloat_t input);

////////////////////////////////    Increment PID    ///////////////////////////////////////////

typedef struct incPID_Typedef {
    incFloat_t kp;
    incFloat_t ki;
    incFloat_t kd;

    struct ctr {
        incFloat_t cur;
        incFloat_t pre;
        incFloat_t aim;
    } ctr;

    incFloat_t e[3]
} incPID_t;

/**
 * @brief 增量式PID初始化函数
 * @param pid pid句柄
 * @param kp 比例参数
 * @param ki 积分参数
 * @param kd 微分参数
 */
void PID_InitInc(incPID_t* pid, incFloat_t kp, incFloat_t ki, incFloat_t kd);

/**
 * @brief 增量式PID设定目标值
 * @param pid pid句柄
 * @param aim 目标值
 */
void PID_SetAimInc(incPID_t* pid, incFloat_t aim);

/**
 * @brief 增量式PID重置函数
 * @param pid pid句柄
 * @note 重置函数不对PID参数和目标值进行重置，仅对运算中间量进行重置
 */
void PID_ResetInc(incPID_t* pid);

/**
 * @brief 增量式PID实现函数
 * @param pid pid句柄
 * @param input 当前输入量
 * @return PID输出量
 */
float PID_RealizeInc(incPID_t* pid, incFloat_t input);

#endif // PID_H