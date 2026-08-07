/*
 * test_setpoint.c —— SetpointBuffer（§11）：Slow 写 → Fast 读
 */
#include "runtime/setpoint.h"
#include "../test_assert.h"

int main(void)
{
    SetpointBuffer sb;
    ControlSetpoint sp;
    ControlSetpoint out;

    sb.index = 0u;
    sb.last_index = 0u;

    sp.voltage_q = 5.0f;
    sp.voltage_d = 0.0f;
    sp.current_q = 0.0f;
    sp.torque    = 0.0f;
    sp.seq       = 1u;

    setpoint_write(&sb, &sp);
    CHECK(setpoint_read(&sb, &out));               /* 新设定值 */
    CHECK_NEAR(out.voltage_q, 5.0f, 1e-6f);

    /* 保持型：无新写 → 最近值 */
    CHECK(!setpoint_read(&sb, &out));
    CHECK_NEAR(out.voltage_q, 5.0f, 1e-6f);

    /* 覆盖写 */
    sp.voltage_q = 8.0f;
    sp.seq = 2u;
    setpoint_write(&sb, &sp);
    CHECK(setpoint_read(&sb, &out));
    CHECK_NEAR(out.voltage_q, 8.0f, 1e-6f);

    TEST_REPORT();
}
