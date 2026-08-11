/*
 * current_controller.h —— V0.2 电流环级联控制器（§10.2.1 V0.2 语义）
 *
 * 控制链（V0.2）：Position P → Velocity PI → iq_sp → Current PI → Voltage → SVPWM
 * 对照三家：
 *   · ODrive：current_control_bandwidth → 增益推导（kp=bw·L, ki=bw·R）；Vd/Vq 前馈；调制饱和锁积分
 *   · SimpleFOC：currentPID（P=L·2π·bw, I=R·2π·bw）+ LPF_current_q + feedforward
 *   · VESC：foc_current_kp/ki + BEMF 前馈（ωe·flux）+ 交叉解耦
 * 本实现：dq 电流 PI（q 环 + d 环）＋ BEMF 前馈 ＋ 交叉解耦 ＋ 一阶电流低通 ＋
 *         带宽自动推导（cfg.current_bandwidth_hz；cfg.current_kp/ki >0 可覆盖）。
 * step_slow(1kHz)：位置/速度环 → iq_sp（写入 sp->current_q，替代 V0.1 的 voltage_q）
 * step_fast(20kHz)：fb.ia/ib → Clarke/Park → 电流 PI → voltage_q/d（替代 V0.1 电压直通）
 */
#ifndef FOC_CONTROL_CURRENT_CONTROLLER_H
#define FOC_CONTROL_CURRENT_CONTROLLER_H

#include <stdint.h>
#include "control/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 位置 P + 速度 PI（Slow 环，1kHz）：增益取自 cfg.kp/ki/kd（快照） */
    float   integral;            /* 速度积分累加 */
    float   integral_limit;      /* 防 windup */
    float   kp_pos;              /* 位置误差 → 速度目标 [1/s] */
    float   vel_limit_from_pos;  /* 位置环输出的速度目标限幅 [rad/s] */
    ControlMode active_mode;
    float   last_velocity_target;

    /* 电流环（Fast 环，20kHz） */
    float   vq_integral;         /* 电流 PI 积分 [V] */
    float   vd_integral;
    float   iq_lpf;              /* 电流一阶低通 */
    float   id_lpf;
    float   max_current;         /* iq 目标绝对上限 [A]（注入自 cap.max_current） */
    float   v_limit;             /* 输出电压上限 [V]（积分限幅/防 windup） */
    bool    first_fast;

    /* 电机参数（注入自 scfg，ODrive 式增益推导） */
    float   pole_pairs;
    float   phase_resistance;    /* [Ω] */
    float   phase_inductance;    /* [H] */
    float   flux_linkage;        /* 反电动势常数 [V·s/rad]（= ke，BEMF 前馈） */

    /* 诊断（只读） */
    float   last_iq_measured;
    float   last_id_measured;
    float   last_vq, last_vd;
} CurrentControllerCtx;

/* 静态绑定（§10.2 收口）：const ControllerOps，不做注册表 */
extern const ControllerOps current_controller_ops;

void current_controller_init_ctx(CurrentControllerCtx *ctx);

/* 电流环增益推导（ODrive 式）：kp = bw·L，ki = bw·R（bw [Hz]，R [Ω]，L [H]）。
 * board/用户可用此函数按带宽自动算出 kp/ki 填 cfg；cfg.current_kp/ki>0 时优先用直接值 */
void current_controller_derive_gains(float bw_hz, float R, float L, float *kp, float *ki);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_CURRENT_CONTROLLER_H */
