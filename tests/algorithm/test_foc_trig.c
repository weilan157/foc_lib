/*
 * test_foc_trig.c —— 三角实现单测（查表+插值 默认 / 多项式 / 硬件后端的算法级验证）
 *
 * 覆盖：特殊角、与标准库精度对比、周期性、负角、s²+c²=1、便捷单值接口。
 * 注意：默认 FOC_TRIG_IMPL=LOOKUP（误差 ~3e-5）；切 POLY 时误差 ~1e-3，
 *       本测试统一按 1e-3 容差断言（对 FOC 足够，坐标变换往返容差 1e-4）。
 */
#include "foc/foc_trig.h"
#include "../test_assert.h"
#include <math.h>

#define PI     (3.141592653589793f)
#define TWO_PI (6.283185307179586f)

int main(void)
{
    /* 特殊角（表点，应接近精确） */
    {
        float s, c;
        foc_sincos(0.0f, &s, &c);
        CHECK_NEAR(s, 0.0f, 1e-5f);
        CHECK_NEAR(c, 1.0f, 1e-5f);

        foc_sincos(PI / 2.0f, &s, &c);
        CHECK_NEAR(s, 1.0f, 1e-5f);
        CHECK_NEAR(c, 0.0f, 1e-5f);

        foc_sincos(PI, &s, &c);
        CHECK_NEAR(s, 0.0f, 1e-4f);
        CHECK_NEAR(c, -1.0f, 1e-5f);

        foc_sincos(3.0f * PI / 2.0f, &s, &c);
        CHECK_NEAR(s, -1.0f, 1e-5f);
        CHECK_NEAR(c, 0.0f, 1e-4f);
    }

    /* 与标准库比较：0..2π 扫 720 点，最大误差 < 1e-3（FOC 足够） */
    {
        int i;
        float max_err = 0.0f;

        for (i = 0; i <= 720; i++) {
            float th = (float)i * TWO_PI / 720.0f;
            float s, c;
            float es, ec;

            foc_sincos(th, &s, &c);
            es = fabsf(s - sinf(th));
            ec = fabsf(c - cosf(th));
            if (es > max_err) { max_err = es; }
            if (ec > max_err) { max_err = ec; }
        }
        /* 误差界：LOOKUP ~3e-5 / CORDIC ~1e-7；POLY（VESC Bhaskara）~1.1e-3。统一按 2e-3 容差 */
        CHECK(max_err < 2e-3f);
    }

    /* 周期性：x 与 x + 2π 一致 */
    {
        float s1, c1, s2, c2;
        foc_sincos(0.7f, &s1, &c1);
        foc_sincos(0.7f + TWO_PI, &s2, &c2);
        CHECK_NEAR(s1, s2, 1e-5f);
        CHECK_NEAR(c1, c2, 1e-5f);
    }

    /* 负角：sin(-x) = -sin(x)，cos(-x) = cos(x) */
    {
        float s1, c1, s2, c2;
        foc_sincos(0.9f, &s1, &c1);
        foc_sincos(-0.9f, &s2, &c2);
        CHECK_NEAR(s1, -s2, 1e-5f);
        CHECK_NEAR(c1,  c2, 1e-5f);
    }

    /* 恒等式：s² + c² ≈ 1 */
    {
        int i;
        for (i = 0; i < 32; i++) {
            float th = (float)i * 0.2f;
            float s, c;
            foc_sincos(th, &s, &c);
#if (FOC_TRIG_IMPL == FOC_TRIG_IMPL_POLY)
            /* 独立近似，正交性偏差更大，放宽到 5e-3 */
            CHECK(fabsf((s * s) + (c * c) - 1.0f) < 5e-3f);
#else
            CHECK(fabsf((s * s) + (c * c) - 1.0f) < 1e-4f);
#endif
        }
    }

    /* 便捷单值接口 */
    CHECK_NEAR(foc_sin(PI / 2.0f), 1.0f, 1e-5f);
    CHECK_NEAR(foc_cos(0.0f), 1.0f, 1e-5f);

    TEST_REPORT();
}
