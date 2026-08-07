/*
 * motor_model.c —— 仿真电机模型实现
 * 简化一阶机械 + 稳态电气（忽略 L）：
 *   Park(αβ→dq) → iq = (vq - ke*ω_elec)/R → 转矩 = kt*iq → J·α = τ - b·ω → 积分位置/速度
 */
#include <math.h>
#include "simulation/motor_model.h"
#include "foc_types.h"

void motor_model_init(MotorModel *m, const MotorStaticConfig *scfg)
{
    if (m == NULL) { return; }

    m->R        = (scfg != NULL) ? scfg->phase_resistance : 1.0f;
    m->L        = (scfg != NULL) ? scfg->phase_inductance : 0.0001f;
    m->pole_pairs = (scfg != NULL) ? (float)scfg->pole_pairs : 7.0f;
    m->ke        = 0.01f;            /* 小 BLDC 反电动势系数 */
    m->kt        = m->ke * 1.5f * m->pole_pairs;   /* 简化转矩系数 */
    m->J         = 0.001f;           /* [kg·m²] */
    m->b         = 0.0005f;          /* 粘滞摩擦 */
    m->vbus      = 24.0f;

    m->mech_angle = 0.0f;
    m->mech_vel   = 0.0f;
    m->iq         = 0.0f;
    m->timestamp_us = 0u;
    m->cycle      = 0u;
}

int motor_model_step(MotorModel *m, const VoltageVector *vv, float dt)
{
    float elec_angle;
    float vd, vq;
    float emf;
    float torque;
    float accel;
    float iq_lim;

    if (m == NULL || vv == NULL) { return FOC_ERROR; }
    if (dt <= 0.0f) { return FOC_OK; }

    elec_angle = m->mech_angle * m->pole_pairs;

    /* Park：αβ 电压 → dq */
    vd =  vv->alpha_v * cosf(elec_angle) + vv->beta_v * sinf(elec_angle);
    vq = -vv->alpha_v * sinf(elec_angle) + vv->beta_v * cosf(elec_angle);
    (void)vd;   /* V0.1 只用 q 轴电压驱动 */

    /* 稳态电气（忽略 L）：iq = (vq - ke*ω_elec) / R */
    emf = m->ke * (m->mech_vel * m->pole_pairs);
    iq_lim = (m->vbus / (2.0f * m->R)) * 3.0f;   /* 粗略电流限幅防发散 */
    m->iq = (vq - emf) / m->R;
    if (m->iq >  iq_lim) { m->iq =  iq_lim; }
    if (m->iq < -iq_lim) { m->iq = -iq_lim; }

    /* 机械：J·α = τ - b·ω */
    torque = m->kt * m->iq;
    accel  = (torque - m->b * m->mech_vel) / m->J;

    m->mech_vel   += accel * dt;
    m->mech_angle += m->mech_vel * dt;

    /* 角度规范化到 [-pi, pi) 防累计漂移（圈数由 revolution 累计，测试用单圈） */
    if (m->mech_angle >  (float)M_PI) { m->mech_angle -= 2.0f * (float)M_PI; }
    if (m->mech_angle < -(float)M_PI) { m->mech_angle += 2.0f * (float)M_PI; }

    m->cycle++;
    return FOC_OK;
}

void motor_model_get_feedback(const MotorModel *m, EncoderFeedback *fb)
{
    if (m == NULL || fb == NULL) { return; }
    fb->mech_angle_rad = m->mech_angle;
    fb->revolution     = 0;
    fb->velocity       = m->mech_vel;
    fb->quality        = ENC_QUALITY_GOOD;
}
