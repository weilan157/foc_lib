/*
 * timebase.h —— 实时基础设施（§4.5，冻结）
 * Fast/Slow Loop 任务头统一 timebase_update()；dt 必须实测，禁硬编码。
 * 单一时间源（方案 B 收口）：timebase_get_step(tb, &step) 以传入 tb 为单一输入。
 */
#ifndef FOC_RUNTIME_TIMEBASE_H
#define FOC_RUNTIME_TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t cycle;         /* 周期计数 */
    uint64_t timestamp_us;  /* 当前时刻 [µs] */
    float    dt;            /* 实测周期 [s]（由 update 计算，clamp 到 [0, dt_max]） */
} TimeBase;

/* 消费者冻结视图：控制链不直接摸 TimeBase 原始字段 */
typedef struct {
    float    dt;             /* 实测周期 [s] */
    uint32_t overrun_count;  /* 超执行预算累计（V0.1 由调用方/RuntimeStats 维护） */
    bool     valid;          /* dt 有效（首次 / 异常为 false，消费方跳过该周期） */
} TimeStep;

void timebase_init(TimeBase *tb, uint64_t now_us);
void timebase_update(TimeBase *tb, uint64_t now_us, float dt_max_s); /* 任务头调用 */
bool timebase_get_step(const TimeBase *tb, TimeStep *step);          /* 控制链调用 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_TIMEBASE_H */
