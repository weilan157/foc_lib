/*
 * test_timebase.c —— TimeBase/TimeStep（§4.5）：dt 实测 + clamp + valid
 */
#include "runtime/timebase.h"
#include "../test_assert.h"

int main(void)
{
    TimeBase tb;

    timebase_init(&tb, 1000000u);   /* t=1s */

    /* 首周期：dt=0（无上次时间 → valid=false） */
    {
        TimeStep s;
        timebase_update(&tb, 1000000u + 500u, 0.001f);   /* 500µs 后 */
        CHECK(timebase_get_step(&tb, &s));
        CHECK_NEAR(s.dt, 0.0005f, 1e-9f);
        CHECK(s.valid);
    }

    /* 超过 dt_max → clamp */
    {
        TimeStep s;
        timebase_update(&tb, 1000000u + 10000u, 0.001f); /* 9.5ms 周期 > 1ms max */
        CHECK(timebase_get_step(&tb, &s));
        CHECK_NEAR(s.dt, 0.001f, 1e-9f);                 /* clamp 到 dt_max */
        CHECK(s.valid);
    }

    /* cycle 递增 */
    CHECK(tb.cycle == 2u);

    TEST_REPORT();
}
