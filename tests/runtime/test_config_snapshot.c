/*
 * test_config_snapshot.c —— ConfigSnapshot（§10.1，必修 1/4）：一致性快照读
 */
#include "runtime/config_snapshot.h"
#include "../test_assert.h"

int main(void)
{
    ConfigSnapshot cs;
    MotorRuntimeConfig cfg;
    MotorRuntimeConfig out;

    cfg.limits.limit[CTRL_MODE_TORQUE]   = (Limit){ .max = 12.0f,  .min = -12.0f };
    cfg.limits.limit[CTRL_MODE_VELOCITY] = (Limit){ .max = 50.0f,  .min = -50.0f };
    cfg.limits.limit[CTRL_MODE_POSITION] = (Limit){ .max = 6.28f,  .min = -6.28f };
    cfg.kp = 0.5f;
    cfg.ki = 1.0f;
    cfg.kd = 0.0f;

    config_snapshot_init(&cs, &cfg);
    CHECK(config_snapshot_read(&cs, &out));
    CHECK_NEAR(out.kp, 0.5f, 1e-6f);
    CHECK_NEAR(out.limits.limit[CTRL_MODE_TORQUE].max, 12.0f, 1e-6f);

    /* 在线修改 → 快照切换生效 */
    cfg.kp = 2.0f;
    cfg.ki = 3.0f;
    config_snapshot_update(&cs, &cfg);
    CHECK(config_snapshot_read(&cs, &out));
    CHECK_NEAR(out.kp, 2.0f, 1e-6f);
    CHECK_NEAR(out.ki, 3.0f, 1e-6f);

    TEST_REPORT();
}
