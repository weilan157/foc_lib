/*
 * pid.c —— 纯 PID 算法（状态在调用方 PidCtx）
 *
 * - 积分项先按 integral_limit 限幅（anti-windup）
 * - 输出按 output_limit 限幅
 * - 微分作用在误差上（标准形式）
 */
#include "foc/pid.h"

void pid_reset(PidCtx *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->integral   = 0.0f;
    ctx->prev_error = 0.0f;
    ctx->first      = true;
}

float pid_update(PidCtx *ctx, const PidParam *p, float setpoint, float feedback)
{
    float err;
    float p_term;
    float d_term;
    float out;

    if ((ctx == NULL) || (p == NULL) || (p->dt <= 0.0f)) {
        return 0.0f;
    }

    err = setpoint - feedback;

    if (ctx->first) {
        ctx->prev_error = err;
        ctx->first      = false;
    }

    /* P */
    p_term = p->kp * err;

    /* I（先累加再限幅，防 windup） */
    ctx->integral += p->ki * err * p->dt;
    ctx->integral  = foc_clampf(ctx->integral, -p->integral_limit, p->integral_limit);

    /* D（误差微分） */
    d_term = p->kd * (err - ctx->prev_error) / p->dt;
    ctx->prev_error = err;

    out = p_term + ctx->integral + d_term;
    out = foc_clampf(out, -p->output_limit, p->output_limit);
    return out;
}
