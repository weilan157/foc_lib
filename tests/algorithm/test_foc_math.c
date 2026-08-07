/*
 * test_foc_math.c —— FOC 坐标变换单测（Clarke / Park / 逆Park / 电角度）
 */
#include "foc/foc_math.h"
#include "../test_assert.h"

#define SQRT3 (1.7320508075688772f)
#define PI    (3.141592653589793f)

int main(void)
{
    /* Clarke：ia=1, ib=0, ic=-1（ia+ib+ic=0）→ alpha=1, beta=1/sqrt3 */
    {
        AlphaBeta ab = foc_clarke(1.0f, 0.0f, -1.0f);
        CHECK_NEAR(ab.alpha, 1.0f, 1e-5f);
        CHECK_NEAR(ab.beta, 1.0f / SQRT3, 1e-5f);
    }

    /* Clarke：ia=0, ib=1, ic=-1 → alpha=0, beta=2/sqrt3 */
    {
        AlphaBeta ab = foc_clarke(0.0f, 1.0f, -1.0f);
        CHECK_NEAR(ab.alpha, 0.0f, 1e-5f);
        CHECK_NEAR(ab.beta, 2.0f / SQRT3, 1e-5f);
    }

    /* Park：theta=0，alpha=1, beta=0 → d=1, q=0 */
    {
        Dq dq = foc_park(1.0f, 0.0f, 0.0f);
        CHECK_NEAR(dq.d, 1.0f, 1e-5f);
        CHECK_NEAR(dq.q, 0.0f, 1e-5f);
    }

    /* Park：theta=pi/2，alpha=1, beta=0 → d=0, q=-1（约定符号） */
    {
        Dq dq = foc_park(1.0f, 0.0f, PI / 2.0f);
        CHECK_NEAR(dq.d, 0.0f, 1e-5f);
        CHECK_NEAR(dq.q, -1.0f, 1e-5f);
    }

    /* 逆 Park：theta=0，vd=1, vq=0 → alpha=1, beta=0 */
    {
        VoltageVector vv = foc_inverse_park(0.0f, 0.0f, 1.0f, 0.0f);
        CHECK_NEAR(vv.alpha_v, 1.0f, 1e-5f);
        CHECK_NEAR(vv.beta_v, 0.0f, 1e-5f);
    }

    /* 逆 Park：theta=pi/2，vd=1, vq=0 → alpha=0, beta=1 */
    {
        VoltageVector vv = foc_inverse_park(PI / 2.0f, 0.0f, 1.0f, 0.0f);
        CHECK_NEAR(vv.alpha_v, 0.0f, 1e-5f);
        CHECK_NEAR(vv.beta_v, 1.0f, 1e-5f);
    }

    /* 逆 Park↔Park 往返：dq=(1,0.5) 在 theta=0.7 处往返应回到原点 */
    {
        float th = 0.7f;
        VoltageVector vv = foc_inverse_park(th, 0.0f, 1.0f, 0.5f);
        Dq dq = foc_park(vv.alpha_v, vv.beta_v, th);
        CHECK_NEAR(dq.d, 1.0f, 1e-4f);
        CHECK_NEAR(dq.q, 0.5f, 1e-4f);
    }

    /* 电角度：mech=1.0, pp=7, zero=0.5 → 7.5 规范到 [-pi, pi) */
    {
        float elec = foc_calc_elec_angle(1.0f, 7u, 0.5f);
        float expected = 7.5f - (2.0f * PI); /* 7.5 - 2pi ≈ 1.2168 */
        CHECK_NEAR(elec, expected, 1e-4f);
    }

    /* wrap_pi 边界：pi → -pi（结果为 [-pi, pi)） */
    {
        float w = foc_wrap_pi(PI);
        CHECK_NEAR(w, -PI, 1e-4f);
        w = foc_wrap_pi(-PI);
        CHECK_NEAR(w, -PI, 1e-4f);
        w = foc_wrap_pi(0.0f);
        CHECK_NEAR(w, 0.0f, 1e-5f);
    }

    /* 编码器增大 = 电角度增大（约定核对：对 mech 求导为正） */
    {
        float e0 = foc_calc_elec_angle(1.0f, 4u, 0.0f);
        float e1 = foc_calc_elec_angle(1.1f, 4u, 0.0f);
        CHECK(e1 > e0);
    }

    TEST_REPORT();
}
