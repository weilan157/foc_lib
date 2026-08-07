/*
 * test_limiter.c —— 输出 Limiter（§14）：cap 限幅电压
 */
#include "runtime/limiter.h"
#include "../test_assert.h"

int main(void)
{
    MotorCapability cap = { .max_voltage = 12.0f, .max_current = 3.0f,
                            .max_speed = 50.0f, .max_torque = 0.5f };
    MotorRuntimeConfig rcfg = { 0 };
    ControlOutput out;

    /* 超限 → 夹到 cap */
    out.voltage_q = 20.0f;
    out.voltage_d = -15.0f;
    limiter_apply(&out, &cap, &rcfg);
    CHECK_NEAR(out.voltage_q, 12.0f, 1e-6f);
    CHECK_NEAR(out.voltage_d, -12.0f, 1e-6f);

    /* 未超限 → 不变 */
    out.voltage_q = 5.0f;
    out.voltage_d = -3.0f;
    limiter_apply(&out, &cap, &rcfg);
    CHECK_NEAR(out.voltage_q, 5.0f, 1e-6f);
    CHECK_NEAR(out.voltage_d, -3.0f, 1e-6f);

    TEST_REPORT();
}
