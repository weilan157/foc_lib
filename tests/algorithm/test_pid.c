/*
 * test_pid.c —— PID 单测（P / I 累加 / anti-windup / 输出限幅 / reset）
 */
#include "foc/pid.h"
#include "../test_assert.h"

int main(void)
{
    PidCtx ctx;
    PidParam p;

    p.kp = 2.0f; p.ki = 0.0f; p.kd = 0.0f;
    p.integral_limit = 100.0f; p.output_limit = 100.0f; p.dt = 0.001f;

    /* P-only：err=10 → out=20 */
    pid_reset(&ctx);
    CHECK_NEAR(pid_update(&ctx, &p, 10.0f, 0.0f), 20.0f, 1e-4f);

    /* 输出限幅：err=60 → 120 被夹到 100 */
    p.output_limit = 100.0f;
    pid_reset(&ctx);
    CHECK_NEAR(pid_update(&ctx, &p, 60.0f, 0.0f), 100.0f, 1e-4f);

    /* I-only：ki=1, dt=0.001, err 恒为 1 → 100 步后积分 ≈ 0.1 */
    p.kp = 0.0f; p.ki = 1.0f; p.output_limit = 1000.0f; p.integral_limit = 1000.0f;
    pid_reset(&ctx);
    {
        int i;
        float out = 0.0f;
        for (i = 0; i < 100; i++) {
            out = pid_update(&ctx, &p, 1.0f, 0.0f);
        }
        CHECK_NEAR(out, 0.1f, 1e-4f);
    }

    /* anti-windup：integral_limit=0.05，长期饱和时积分不超限 */
    p.integral_limit = 0.05f;
    pid_reset(&ctx);
    {
        int i;
        float out = 0.0f;
        for (i = 0; i < 10000; i++) {
            out = pid_update(&ctx, &p, 1.0f, 0.0f);
        }
        CHECK_NEAR(out, 0.05f, 1e-4f); /* 夹在 integral_limit */
    }

    /* D 项：kd=1, dt=0.001，误差从 0→1 → d=(1-0)/0.001=1000 */
    p.kp = 0.0f; p.ki = 0.0f; p.kd = 1.0f;
    p.integral_limit = 0.0f; p.output_limit = 5000.0f; p.dt = 0.001f;
    pid_reset(&ctx);
    (void)pid_update(&ctx, &p, 0.0f, 0.0f); /* 首次：prev_error=0 */
    CHECK_NEAR(pid_update(&ctx, &p, 1.0f, 0.0f), 1000.0f, 1e-3f);

    /* reset：清积分 */
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.integral_limit = 1000.0f; p.output_limit = 1000.0f; p.dt = 0.001f;
    pid_reset(&ctx);
    (void)pid_update(&ctx, &p, 1.0f, 0.0f);
    pid_reset(&ctx);
    CHECK_NEAR(pid_update(&ctx, &p, 1.0f, 0.0f), 0.001f, 1e-6f); /* 积分从 0 重新累积 */

    TEST_REPORT();
}
