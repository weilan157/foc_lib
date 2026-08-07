/*
 * stats.c —— RuntimeStats（§4.9，冻结）
 * 现场"偶尔抖一下"必须可查；diag_get_stats() 供上位机读取。
 */
#include "runtime/stats.h"

void stats_init(RuntimeStats *s)
{
    if (s == NULL) { return; }
    s->loop_count   = 0u;
    s->max_exec_us  = 0u;
    s->overrun_count = 0u;
    s->min_dt_us    = 0u;
    s->max_dt_us    = 0u;
}

void stats_update(RuntimeStats *s, const TimeBase *tb)
{
    uint32_t dt_us;

    if (s == NULL || tb == NULL) { return; }

    s->loop_count++;
    dt_us = (uint32_t)(tb->dt * 1000000.0f);

    if (s->loop_count == 1u) {
        s->min_dt_us = dt_us;
        s->max_dt_us = dt_us;
    } else {
        if (dt_us < s->min_dt_us) { s->min_dt_us = dt_us; }
        if (dt_us > s->max_dt_us) { s->max_dt_us = dt_us; }
    }
}

void stats_update_exec(RuntimeStats *s, uint32_t exec_us, uint32_t budget_us)
{
    if (s == NULL) { return; }

    if (exec_us > s->max_exec_us) { s->max_exec_us = exec_us; }
    if (exec_us > budget_us)      { s->overrun_count++; }
}
