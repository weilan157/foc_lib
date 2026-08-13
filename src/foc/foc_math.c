/*
 * foc_math.c —— FOC 坐标变换（纯数学）
 *
 * 坐标系约定（§3.6，写死）：
 *   Clarke：ia + ib + ic = 0
 *   Park/逆Park：编码器增大 = 电角度增大；正 Iq = 正转矩
 */
#include "foc/foc_math.h"
#include "foc/foc_trig.h"   /* sin/cos 走 foc_sincos（查表+插值默认；CORDIC 可经 FOC_TRIG_IMPL 切换） */
#include <math.h>

#define FOC_SQRT3   (1.7320508075688772f) /* sqrt(3) */
#define FOC_INV_SQRT3 (0.5773502691896258f) /* 1/sqrt(3)：乘法替代除法（Fast Loop 热点） */
#define FOC_TWO_PI  (6.283185307179586f)

AlphaBeta foc_clarke(float ia, float ib, float ic)
{
    AlphaBeta ab;
    (void)ic; /* 利用 ia+ib+ic=0，ic 仅作校验/记录 */

    /* alpha = ia；beta = (ia + 2*ib)/sqrt(3)（由 ic = -ia-ib 代入标准 Clarke 得） */
    ab.alpha = ia;
    ab.beta  = (ia + (2.0f * ib)) * FOC_INV_SQRT3;
    return ab;
}

Dq foc_park(float alpha, float beta, float theta_rad)
{
    Dq dq;
    float c;
    float s;

    foc_sincos(theta_rad, &s, &c);   /* 查表+插值（默认）/ 多项式 / 硬件 CORDIC（FOC_TRIG_IMPL） */

    dq.d = (alpha * c) + (beta * s);
    dq.q = (-alpha * s) + (beta * c);
    return dq;
}

VoltageVector foc_inverse_park(float theta_rad, float v_zero, float v_d, float v_q)
{
    VoltageVector vv;
    float c;
    float s;

    (void)v_zero; /* V0.1 无零序注入 */

    foc_sincos(theta_rad, &s, &c);   /* 查表+插值（默认）/ 多项式 / 硬件 CORDIC（FOC_TRIG_IMPL） */

    vv.alpha_v = (v_d * c) - (v_q * s);
    vv.beta_v  = (v_d * s) + (v_q * c);
    return vv;
}

float foc_calc_elec_angle(float mech_angle_rad, uint32_t pole_pairs, float encoder_zero)
{
    float elec;
    if (pole_pairs == 0u) {
        pole_pairs = 1u;
    }
    elec = (mech_angle_rad * (float)pole_pairs) + encoder_zero;
    return foc_wrap_pi(elec);
}

float foc_wrap_pi(float a)
{
    /* 快速 wrap 到 [-π, π)：乘 1/2π + float→int 截断取圈数（FPU 单条 vcvt 指令），
       替代 fmodf（无硬件指令、软件循环数百周期，20kHz Fast Loop 的性能杀手）。
       角度圈数有限（< 1e6 圈，int32 不会溢出）；截断向零，负角度下同样正确。 */
    float n = (float)(int32_t)(a * (1.0f / FOC_TWO_PI));
    a -= n * FOC_TWO_PI;
    if (a >= (float)M_PI) {
        a -= FOC_TWO_PI;
    } else if (a < -(float)M_PI) {
        a += FOC_TWO_PI;
    }
    return a;
}
