/*
 * current_controller.c —— V0.2 电流环级联控制器实现
 *
 * Slow(1kHz)：Position P → Velocity PI → iq_sp（写 sp->current_q，限幅）
 * Fast(20kHz)：fb.ia/ib → Clarke/Park → 电流 PI（BEMF 前馈 + 交叉解耦）→ voltage_q/d
 * 增益：cfg.current_kp/ki（>0）优先，否则带宽推导（kp=bw·L, ki=bw·R，ODrive 式）。
 */
#include "control/current_controller.h"
#include "foc/foc_math.h"
#include "foc_types.h"
#include <math.h>

#define CC_TWO_PI (6.283185307179586f)

enum { CC_PARAM_KP_POS = 1u, CC_PARAM_VEL_LIMIT };   /* 与 pid_controller 一致 */

void current_controller_derive_gains(float bw_hz, float R, float L, float *kp, float *ki)
{
    if (kp != NULL) { *kp = bw_hz * L; }   /* ODrive：p_gain = bw·L */
    if (ki != NULL) { *ki = bw_hz * R; }   /* ODrive：i_gain = bw·R（plant pole = R/L） */
}

void current_controller_init_ctx(CurrentControllerCtx *ctx)
{
    if (ctx == NULL) { return; }
    ctx->integral             = 0.0f;
    ctx->integral_limit       = 24.0f;     /* 速度积分上限（电压域） */
    ctx->kp_pos               = 10.0f;
    ctx->vel_limit_from_pos   = 10.0f;
    ctx->active_mode          = CTRL_MODE_VELOCITY;
    ctx->last_velocity_target = 0.0f;

    ctx->vq_integral = 0.0f;
    ctx->vd_integral = 0.0f;
    ctx->iq_lpf = 0.0f;
    ctx->id_lpf = 0.0f;
    ctx->max_current = 3.0f;               /* 默认；board 注入 cap.max_current */
    ctx->v_limit = 24.0f;
    ctx->first_fast = true;

    ctx->pole_pairs = 4u;                  /* 默认；board 注入 scfg */
    ctx->phase_resistance = 1.0f;
    ctx->phase_inductance = 0.001f;
    ctx->flux_linkage = 0.01f;

    ctx->last_iq_measured = 0.0f;
    ctx->last_id_measured = 0.0f;
    ctx->last_vq = 0.0f;
    ctx->last_vd = 0.0f;
}

static int cc_init(void *ctx)
{
    current_controller_init_ctx((CurrentControllerCtx *)ctx);
    return FOC_OK;
}

static int cc_reset(void *ctx)
{
    CurrentControllerCtx *c = (CurrentControllerCtx *)ctx;
    if (c == NULL) { return FOC_ERROR; }
    c->integral = 0.0f;
    c->vq_integral = 0.0f;
    c->vd_integral = 0.0f;
    c->iq_lpf = 0.0f;
    c->id_lpf = 0.0f;
    c->last_velocity_target = 0.0f;
    c->first_fast = true;
    return FOC_OK;
}

static int cc_on_enter(void *ctx, ControlMode mode)
{
    CurrentControllerCtx *c = (CurrentControllerCtx *)ctx;
    if (c == NULL) { return FOC_ERROR; }
    cc_reset(c);
    c->active_mode = mode;
    return FOC_OK;
}

static int cc_on_exit(void *ctx, ControlMode mode)
{
    CurrentControllerCtx *c = (CurrentControllerCtx *)ctx;
    if (c == NULL) { return FOC_ERROR; }
    cc_reset(c);
    (void)mode;
    return FOC_OK;
}

static int cc_set_param(void *ctx, uint32_t param_id, const void *val)
{
    CurrentControllerCtx *c = (CurrentControllerCtx *)ctx;
    const float *f;
    if ((c == NULL) || (val == NULL)) { return FOC_ERROR; }
    f = (const float *)val;
    switch (param_id) {
    case CC_PARAM_KP_POS:    c->kp_pos = *f; break;
    case CC_PARAM_VEL_LIMIT: c->vel_limit_from_pos = *f; break;
    default: return FOC_ERROR;
    }
    return FOC_OK;
}

/* 一阶低通：alpha = dt·wc/(1+dt·wc)（fc<=0 → 直通） */
static float cc_lpf(float in, float prev, float fc_hz, float dt)
{
    float wc, alpha;
    if (fc_hz <= 0.0f) { return in; }
    wc = CC_TWO_PI * fc_hz;
    alpha = (dt * wc) / (1.0f + dt * wc);
    return prev + alpha * (in - prev);
}

/* ---- Slow(1kHz)：位置 P → 速度 PI → iq_sp ---- */
static int cc_step_slow(void *ctx, const MotorCommand *cmd, const FastFeedback *fb,
                        const MotorRuntimeConfig *cfg, float dt, ControlSetpoint *sp)
{
    CurrentControllerCtx *c = (CurrentControllerCtx *)ctx;
    float vel_target;
    float iq_sp;

    if ((c == NULL) || (cmd == NULL) || (fb == NULL) || (cfg == NULL) || (sp == NULL)) {
        return FOC_ERROR;
    }

    /* 位置环 P（POSITION）：误差 → 速度目标（限幅） */
    if (cmd->mode == CTRL_MODE_POSITION) {
        vel_target = c->kp_pos * (cmd->target - fb->mech_angle_rad);
        if (vel_target > c->vel_limit_from_pos)      { vel_target = c->vel_limit_from_pos; }
        else if (vel_target < -c->vel_limit_from_pos){ vel_target = -c->vel_limit_from_pos; }
    } else {
        vel_target = cmd->target;
    }

    if (cmd->mode == CTRL_MODE_TORQUE) {
        iq_sp = cmd->target;                        /* TORQUE：命令即电流目标 [A] */
    } else {
        /* 速度 PI：vel_err → iq_sp */
        float vel_err = vel_target - fb->mech_vel_radps;
        c->integral += vel_err * cfg->ki * dt;
        if (c->integral > c->integral_limit)      { c->integral = c->integral_limit; }
        else if (c->integral < -c->integral_limit){ c->integral = -c->integral_limit; }
        iq_sp = vel_err * cfg->kp + c->integral;
    }
    c->last_velocity_target = vel_target;

    /* iq_sp 限幅：运行限幅表（TORQUE 档，单位 A）+ 能力上限（ctx.max_current） */
    iq_sp = foc_limit(iq_sp, &cfg->limits.limit[CTRL_MODE_TORQUE]);
    if (iq_sp > c->max_current)      { iq_sp = c->max_current; }
    else if (iq_sp < -c->max_current){ iq_sp = -c->max_current; }

    sp->current_q = iq_sp;                          /* V0.2：iq_sp 载体 */
    sp->voltage_q = 0.0f;
    sp->voltage_d = 0.0f;
    sp->torque = 0.0f;
    sp->seq = cmd->sequence;
    return FOC_OK;
}

/* ---- Fast(20kHz)：电流 PI → voltage ---- */
static int cc_step_fast(void *ctx, const FastFeedback *fb, const ControlSetpoint *sp,
                        const MotorRuntimeConfig *cfg, float dt, ControlOutput *out)
{
    CurrentControllerCtx *c = (CurrentControllerCtx *)ctx;
    AlphaBeta ab;
    Dq idq;
    float id, iq;
    float id_sp, iq_sp;
    float ierr_d, ierr_q;
    float kp = 0.0f, ki = 0.0f;
    float we;
    float dec_d, dec_q, ff_q;
    float vq, vd;

    if ((c == NULL) || (fb == NULL) || (sp == NULL) || (cfg == NULL) || (out == NULL)) {
        return FOC_ERROR;
    }

    /* 电流采样（motor_fast_step 已填 fb.ia/ib）→ Clarke → Park */
    ab = foc_clarke(fb->ia, fb->ib, -(fb->ia + fb->ib));
    idq = foc_park(ab.alpha, ab.beta, fb->elec_angle_rad);
    id = idq.d;
    iq = idq.q;

    /* 一阶低通（可选） */
    id = cc_lpf(id, c->id_lpf, cfg->current_filter_hz, dt);
    iq = cc_lpf(iq, c->iq_lpf, cfg->current_filter_hz, dt);
    c->id_lpf = id;
    c->iq_lpf = iq;

    /* 目标：iq_sp（Slow 产）；id_sp = 0（表贴 PMSM） */
    iq_sp = sp->current_q;
    id_sp = 0.0f;

    /* 增益：cfg 直接值 >0 优先，否则带宽推导 */
    if (cfg->current_kp > 0.0f) { kp = cfg->current_kp; }
    if (cfg->current_ki > 0.0f) { ki = cfg->current_ki; }
    if ((kp <= 0.0f) && (ki <= 0.0f) && (cfg->current_bandwidth_hz > 0.0f)) {
        current_controller_derive_gains(cfg->current_bandwidth_hz,
                                        c->phase_resistance, c->phase_inductance, &kp, &ki);
    }

    ierr_d = id_sp - id;
    ierr_q = iq_sp - iq;

    /* 电角速度 → 交叉解耦 + BEMF 前馈（对照 ODrive/VESC） */
    we    = fb->mech_vel_radps * c->pole_pairs;
    dec_d = -we * c->phase_inductance * iq_sp;     /* vd -= ωe·L·iq */
    dec_q =  we * c->phase_inductance * id_sp;     /* vq += ωe·L·id */
    ff_q  =  c->flux_linkage * we;                 /* vq += ke·ωe（BEMF 前馈） */

    /* 电流 PI（+ 前馈 + 解耦） */
    vq = ierr_q * kp + c->vq_integral + ff_q + dec_q;
    vd = ierr_d * kp + c->vd_integral + dec_d;

    /* 积分（输出限幅防 windup：|v| 达限冻结积分） */
    if (fabsf(vq) < c->v_limit) { c->vq_integral += ierr_q * ki * dt; }
    if (fabsf(vd) < c->v_limit) { c->vd_integral += ierr_d * ki * dt; }

    out->voltage_q = vq;
    out->voltage_d = vd;
    out->current_q = 0.0f;
    out->torque    = 0.0f;

    c->last_iq_measured = iq;
    c->last_id_measured = id;
    c->last_vq = vq;
    c->last_vd = vd;
    return FOC_OK;
}

const ControllerOps current_controller_ops = {
    .init      = cc_init,
    .reset     = cc_reset,
    .on_enter  = cc_on_enter,
    .on_exit   = cc_on_exit,
    .set_param = cc_set_param,
    .step_slow = cc_step_slow,
    .step_fast = cc_step_fast,
};
