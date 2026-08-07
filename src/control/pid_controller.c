/*
 * pid_controller.c —— V0.1 电压模式级联控制器（§10.2.1，冻结语义）
 * 控制链：Position P → Velocity PI → Voltage Command(voltage_sp) → Voltage FOC → SVPWM
 * 变量必须 voltage_sp（ControlSetpoint.voltage_q）；禁止 iq_sp/current_q 参与 V0.1。
 * 速度环 PID 增益经 ConfigSnapshot（cfg.kp/ki/kd）；位置 P 增益在 ctx。
 */
#include "control/pid_controller.h"
#include "foc_types.h"

/* set_param 支持的参数 id */
enum {
    PID_PARAM_KP_POS = 1u,
    PID_PARAM_VEL_LIMIT,
};

void pid_controller_init_ctx(PidControllerCtx *ctx)
{
    if (ctx == NULL) { return; }
    ctx->integral           = 0.0f;
    ctx->integral_limit     = 24.0f;     /* 电压上限（能力限幅在 limiter，此处防 windup） */
    ctx->kp_pos             = 10.0f;     /* 位置误差 → 速度目标 [1/s] */
    ctx->vel_limit_from_pos = 10.0f;     /* [rad/s] */
    ctx->active_mode        = CTRL_MODE_VELOCITY;
    ctx->last_velocity_target = 0.0f;
}

static int pid_init(void *ctx)
{
    pid_controller_init_ctx((PidControllerCtx *)ctx);
    return FOC_OK;
}

static int pid_reset(void *ctx)
{
    PidControllerCtx *pc = (PidControllerCtx *)ctx;
    if (pc == NULL) { return FOC_ERROR; }
    pc->integral = 0.0f;
    pc->last_velocity_target = 0.0f;
    return FOC_OK;
}

static int pid_on_enter(void *ctx, ControlMode mode)
{
    PidControllerCtx *pc = (PidControllerCtx *)ctx;
    if (pc == NULL) { return FOC_ERROR; }
    pc->integral = 0.0f;                 /* 进入新模式：清零积分防跳变 */
    pc->active_mode = mode;
    return FOC_OK;
}

static int pid_on_exit(void *ctx, ControlMode mode)
{
    PidControllerCtx *pc = (PidControllerCtx *)ctx;
    if (pc == NULL) { return FOC_ERROR; }
    pc->integral = 0.0f;                 /* 离开旧模式：清零积分 */
    (void)mode;
    return FOC_OK;
}

static int pid_set_param(void *ctx, uint32_t param_id, const void *val)
{
    PidControllerCtx *pc = (PidControllerCtx *)ctx;
    const float *f;

    if (pc == NULL || val == NULL) { return FOC_ERROR; }
    f = (const float *)val;

    switch (param_id) {
    case PID_PARAM_KP_POS:     pc->kp_pos = *f; break;
    case PID_PARAM_VEL_LIMIT:  pc->vel_limit_from_pos = *f; break;
    default: return FOC_ERROR;
    }
    return FOC_OK;
}

/* 速度 PI：err → voltage_sp（输出限幅交 limiter；此处仅积分 anti-windup） */
static float velocity_pi(PidControllerCtx *pc, float err, float kp, float ki, float dt)
{
    float volt;

    pc->integral += ki * err * dt;
    if (pc->integral >  pc->integral_limit) { pc->integral =  pc->integral_limit; }
    if (pc->integral < -pc->integral_limit) { pc->integral = -pc->integral_limit; }

    volt = kp * err + pc->integral;
    return volt;
}

static int pid_step_slow(void *ctx, const MotorCommand *cmd, const FastFeedback *fb,
                         const MotorRuntimeConfig *cfg, float dt_slow, ControlSetpoint *sp)
{
    PidControllerCtx *pc = (PidControllerCtx *)ctx;
    float voltage_sp = 0.0f;

    if (pc == NULL || cmd == NULL || fb == NULL || cfg == NULL || sp == NULL) { return FOC_ERROR; }

    /* cmd.target 已在 motor_slow_step 按模式限幅（cfg.limits） */
    switch (cmd->mode) {
    case CTRL_MODE_TORQUE:
        /* V0.1：TORQUE = 电压指令（voltage_sp），无电流环（§10.2.1） */
        voltage_sp = cmd->target;
        break;

    case CTRL_MODE_VELOCITY:
        voltage_sp = velocity_pi(pc, cmd->target - fb->mech_vel_radps,
                                 cfg->kp, cfg->ki, dt_slow);
        break;

    case CTRL_MODE_POSITION: {
        float vel_target;
        float err_pos = cmd->target - fb->mech_angle_rad;

        vel_target = pc->kp_pos * err_pos;
        if (vel_target >  pc->vel_limit_from_pos) { vel_target =  pc->vel_limit_from_pos; }
        if (vel_target < -pc->vel_limit_from_pos) { vel_target = -pc->vel_limit_from_pos; }
        pc->last_velocity_target = vel_target;

        voltage_sp = velocity_pi(pc, vel_target - fb->mech_vel_radps,
                                 cfg->kp, cfg->ki, dt_slow);
        break;
    }

    default:
        voltage_sp = 0.0f;
        break;
    }

    sp->voltage_q = voltage_sp;   /* voltage_sp 唯一载体 */
    sp->voltage_d = 0.0f;
    sp->current_q = 0.0f;         /* V0.2 预留，V0.1 不赋值 */
    sp->torque    = 0.0f;
    sp->seq++;
    return FOC_OK;
}

static int pid_step_fast(void *ctx, const FastFeedback *fb, const ControlSetpoint *sp,
                         const MotorRuntimeConfig *cfg, float dt_fast, ControlOutput *out)
{
    (void)ctx;
    (void)fb;
    (void)cfg;
    (void)dt_fast;

    if (sp == NULL || out == NULL) { return FOC_ERROR; }

    out->voltage_q = sp->voltage_q;   /* V0.1 电压直通；V0.2 在此插入 Current PI（§10.2.1） */
    out->voltage_d = sp->voltage_d;
    out->current_q = 0.0f;
    out->torque    = 0.0f;
    return FOC_OK;
}

const ControllerOps pid_controller_ops = {
    .init       = pid_init,
    .reset      = pid_reset,
    .on_enter   = pid_on_enter,
    .on_exit    = pid_on_exit,
    .set_param  = pid_set_param,
    .step_slow  = pid_step_slow,
    .step_fast  = pid_step_fast,
};
