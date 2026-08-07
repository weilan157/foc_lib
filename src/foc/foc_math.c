/*
 * foc_math.c —— FOC 坐标变换（纯数学）
 *
 * 坐标系约定（§3.6，写死）：
 *   Clarke：ia + ib + ic = 0
 *   Park/逆Park：编码器增大 = 电角度增大；正 Iq = 正转矩
 */
#include "foc/foc_math.h"
#include <math.h>

#define FOC_SQRT3   (1.7320508075688772f) /* sqrt(3) */
#define FOC_TWO_PI  (6.283185307179586f)

AlphaBeta foc_clarke(float ia, float ib, float ic)
{
    AlphaBeta ab;
    (void)ic; /* 利用 ia+ib+ic=0，ic 仅作校验/记录 */

    /* alpha = ia；beta = (ia + 2*ib)/sqrt(3)（由 ic = -ia-ib 代入标准 Clarke 得） */
    ab.alpha = ia;
    ab.beta  = (ia + (2.0f * ib)) / FOC_SQRT3;
    return ab;
}

Dq foc_park(float alpha, float beta, float theta_rad)
{
    Dq dq;
    float c;
    float s;

    c = cosf(theta_rad);
    s = sinf(theta_rad);

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

    c = cosf(theta_rad);
    s = sinf(theta_rad);

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
    float r = fmodf(a, FOC_TWO_PI);
    if (r >= (float)M_PI) {
        r -= FOC_TWO_PI;
    } else if (r < -(float)M_PI) {
        r += FOC_TWO_PI;
    }
    return r;
}
