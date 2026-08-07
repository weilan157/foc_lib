/*
 * pid.h —— 纯 PID 算法（无状态驻留：状态在调用方 PidCtx）
 *
 * V0.1.6 Architecture Baseline
 * - 算法层禁止：malloc / HAL / RTOS / 全局变量（§3.3）。
 * - 积分限幅（anti-windup）+ 输出限幅。
 * - 状态只存在于调用方（PidCtx），可多个实例各自持有。
 */
#ifndef FOC_PID_H
#define FOC_PID_H

#include <stdbool.h>
#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PID 参数（运行期可调；经 RuntimeConfig / ConfigSnapshot 修改） */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;   /* 积分项限幅（>=0，防 windup） */
    float output_limit;     /* 输出限幅（>=0） */
    float dt;               /* 控制周期 [s] */
} PidParam;

/* PID 状态（调用方持有） */
typedef struct {
    float integral;       /* 积分累加 */
    float prev_error;     /* 上一次误差（微分） */
    bool  first;          /* 首次调用标志 */
} PidCtx;

/* 复位（清积分/微分历史），模式切换 on_enter 时调用 */
void pid_reset(PidCtx *ctx);

/* 单步：out = PID(setpoint - feedback)，内部限幅并更新状态 */
float pid_update(PidCtx *ctx, const PidParam *p, float setpoint, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PID_H */
