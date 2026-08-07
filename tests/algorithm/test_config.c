/*
 * test_config.c —— 参数体系单测（foc_limit / 按模式限幅表 / MotorStaticConfig 含 cap+limits）
 */
#include "foc/config.h"
#include "../test_assert.h"

int main(void)
{
    /* foc_limit 正常区间 */
    {
        Limit lim = { .max = 10.0f, .min = -10.0f };
        CHECK_NEAR(foc_limit(3.0f, &lim), 3.0f, 0.0f);
        CHECK_NEAR(foc_limit(12.0f, &lim), 10.0f, 0.0f);
        CHECK_NEAR(foc_limit(-12.0f, &lim), -10.0f, 0.0f);
    }

    /* foc_limit 反向区间（min>max）防御：仍输出正确夹取 */
    {
        Limit lim = { .max = 5.0f, .min = 10.0f }; /* 配置错误：max<min */
        CHECK_NEAR(foc_limit(7.0f, &lim), 7.0f, 0.0f);
        CHECK_NEAR(foc_limit(20.0f, &lim), 10.0f, 0.0f);
        CHECK_NEAR(foc_limit(-3.0f, &lim), 5.0f, 0.0f);
    }

    /* CommandLimitTable：按模式限幅（CTRL_MODE_MAX 固定） */
    {
        CommandLimitTable tbl;
        tbl.limit[CTRL_MODE_TORQUE]   = (Limit){ .max = 24.0f,  .min = -24.0f };
        tbl.limit[CTRL_MODE_VELOCITY] = (Limit){ .max = 100.0f, .min = -100.0f };
        tbl.limit[CTRL_MODE_POSITION] = (Limit){ .max = 6.28f,  .min = -6.28f };

        CHECK_NEAR(foc_limit(50.0f, &tbl.limit[CTRL_MODE_TORQUE]), 24.0f, 0.0f);
        CHECK_NEAR(foc_limit(150.0f, &tbl.limit[CTRL_MODE_VELOCITY]), 100.0f, 0.0f);
        CHECK_NEAR(foc_limit(3.0f, &tbl.limit[CTRL_MODE_POSITION]), 3.0f, 0.0f);
    }

    /* MotorStaticConfig 必须持有 cap + limits（定稿结构） */
    {
        MotorStaticConfig scfg;
        scfg.pole_pairs = 7u;
        scfg.cap.max_voltage = 24.0f;
        scfg.cap.max_current = 3.0f;
        scfg.cap.max_speed   = 100.0f;
        scfg.cap.max_torque  = 0.5f;
        scfg.limits.limit[CTRL_MODE_TORQUE] = (Limit){ .max = 24.0f, .min = -24.0f };

        CHECK(scfg.pole_pairs == 7u);
        CHECK_NEAR(scfg.cap.max_voltage, 24.0f, 0.0f);
        CHECK_NEAR(scfg.cap.max_current, 3.0f, 0.0f);
        CHECK_NEAR(scfg.limits.limit[CTRL_MODE_TORQUE].max, 24.0f, 0.0f);
    }

    /* MotorRuntimeConfig：运行期生效表（初始复制自 scfg->limits 语义） */
    {
        MotorRuntimeConfig rcfg;
        rcfg.limits.limit[CTRL_MODE_TORQUE] = (Limit){ .max = 24.0f, .min = -24.0f };
        rcfg.kp = 1.0f;
        rcfg.ki = 0.0f;
        rcfg.kd = 0.0f;
        CHECK_NEAR(rcfg.kp, 1.0f, 0.0f);
    }

    TEST_REPORT();
}
