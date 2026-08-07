/*
 * config_snapshot.h —— RuntimeConfig 唯一访问途径（必修 1/4，冻结，§10.1）
 * MotorRuntime 不持有 MotorRuntimeConfig 指针；在线改参只写 ConfigSnapshot。
 * Service 写（低频），Fast/Slow Loop 每周期读一致性快照。
 */
#ifndef FOC_RUNTIME_CONFIG_SNAPSHOT_H
#define FOC_RUNTIME_CONFIG_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>
#include "foc/config.h"   /* MotorRuntimeConfig */

#ifdef __cplusplus
extern "C" {
#endif

/* 双缓冲（单写单读，无锁）。index 必须 32 位对齐访存（配合 volatile）。 */
typedef struct {
    MotorRuntimeConfig data[2];
    volatile uint32_t  index;
} ConfigSnapshot;

void config_snapshot_init(ConfigSnapshot *cs, const MotorRuntimeConfig *cfg); /* 上电初始化（复制一份到活动槽） */
void config_snapshot_update(ConfigSnapshot *cs, const MotorRuntimeConfig *cfg); /* Service 写 */
bool config_snapshot_read(ConfigSnapshot *cs, MotorRuntimeConfig *out);         /* Fast/Slow 消费 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_CONFIG_SNAPSHOT_H */
