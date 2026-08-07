/*
 * timebase.c —— TimeBase + TimeStep（§4.5，冻结）
 * dt 必须实测（由 timestamp 差分），禁硬编码；异常 clamp 到 [0, dt_max]。
 */
#include "runtime/timebase.h"

void timebase_init(TimeBase *tb, uint64_t now_us)
{
    if (tb == NULL) { return; }
    tb->cycle        = 0u;
    tb->timestamp_us = now_us;
    tb->dt           = 0.0f;
}

void timebase_update(TimeBase *tb, uint64_t now_us, float dt_max_s)
{
    float dt;

    if (tb == NULL) { return; }

    if ((tb->timestamp_us != 0u) && (now_us > tb->timestamp_us)) {
        dt = (float)(now_us - tb->timestamp_us) / 1000000.0f;
    } else {
        dt = 0.0f;   /* 首次或时钟回绕：不计周期 */
    }

    if (dt > dt_max_s) { dt = dt_max_s; }
    if (dt < 0.0f)     { dt = 0.0f; }

    tb->dt           = dt;
    tb->timestamp_us = now_us;
    tb->cycle        = (tb->cycle == 0xFFFFFFFFu) ? 0u : (tb->cycle + 1u);
}

bool timebase_get_step(const TimeBase *tb, TimeStep *step)
{
    if (tb == NULL || step == NULL) { return false; }

    step->dt            = tb->dt;
    step->valid         = (tb->dt > 0.0f);
    step->overrun_count = 0u;   /* V0.1：超预算计数由 RuntimeStats 维护 */
    return step->valid;
}
