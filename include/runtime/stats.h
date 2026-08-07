/*
 * stats.h —— 性能统计（§4.9，冻结）
 * 现场"偶尔抖一下"必须可查。Fast Loop 末尾 stats_update()。
 */
#ifndef FOC_RUNTIME_STATS_H
#define FOC_RUNTIME_STATS_H

#include <stdint.h>
#include "runtime/timebase.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t loop_count;
    uint32_t max_exec_us;     /* 最大执行时间 [µs] */
    uint32_t overrun_count;   /* 超执行预算次数 */
    uint32_t min_dt_us;       /* dt 极值 [µs] */
    uint32_t max_dt_us;
} RuntimeStats;

void stats_init(RuntimeStats *s);
void stats_update(RuntimeStats *s, const TimeBase *tb);              /* Fast Loop 末尾：记录 dt 极值 */
void stats_update_exec(RuntimeStats *s, uint32_t exec_us, uint32_t budget_us); /* 记录执行时间 + 超预算计数 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_STATS_H */
