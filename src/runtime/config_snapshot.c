/*
 * config_snapshot.c —— ConfigSnapshot（§10.1，必修 1/4，冻结）
 * Service 写（低频），Fast/Slow 多读者。read 返回最近一致快照；
 * 未 init（冷启动）→ 返回 false，调用方用 scfg 默认表兜底。
 */
#include "runtime/config_snapshot.h"
#include "foc_types.h"

#include <stdatomic.h>

void config_snapshot_init(ConfigSnapshot *cs, const MotorRuntimeConfig *cfg)
{
    if (cs == NULL) { return; }
    if (cfg != NULL) {
        cs->data[0] = *cfg;
        cs->data[1] = *cfg;
    }
    cs->index = 0u;
}

void config_snapshot_update(ConfigSnapshot *cs, const MotorRuntimeConfig *cfg)
{
    uint32_t active;
    uint32_t inactive;

    if (cs == NULL || cfg == NULL) { return; }

    active   = cs->index;
    inactive = active ^ 1u;

    cs->data[inactive] = *cfg;
    atomic_thread_fence(memory_order_release);
    cs->index = inactive;
}

bool config_snapshot_read(ConfigSnapshot *cs, MotorRuntimeConfig *out)
{
    uint32_t active;

    if (cs == NULL || out == NULL) { return false; }

    active = cs->index;
    *out   = cs->data[active];   /* 一致性快照 */
    return true;
}
