/*
 * safety.h —— 独立 Safety 层（§15，冻结）
 * 安全逻辑不依赖控制算法；即使 Controller 挂了，Safety 仍能关断。
 * FaultManager 不得直接操作 PWM——唯一出口 motor_enter_safe_state()（§16.1）。
 */
#ifndef FOC_SAFETY_SAFETY_H
#define FOC_SAFETY_SAFETY_H

#include <stdint.h>
#include <stdbool.h>
#include "safety/fault.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { SAFE_OK=0, SAFE_STO, SAFE_ESTOP, SAFE_FAULT_LATCH } SafetyState;

typedef struct {
    void (*hal_set_sto)(bool engage);
    void (*hal_set_estop)(bool engage);
    void (*hal_wdt_feed)(void);
} HalSafetyOps;

typedef struct {
    SafetyState      state;
    FaultReg        *fault;
    const HalSafetyOps *hw;
    uint32_t         wdt_timeout_us;
    uint64_t         last_feed_us;
} Safety;

void safety_init(Safety *s);
void safety_enter_sto(Safety *s);          /* 安全转矩关闭（STO 引脚 + PWM disable） */
void safety_enter_estop(Safety *s);        /* 急停（立即关断） */
void safety_watchdog_feed(Safety *s, uint64_t now_us);  /* Fast Loop 每周期喂 */
void safety_check(Safety *s, uint64_t now_us);          /* WDT 超时 / 急停 → 关断 + 锁存 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_SAFETY_SAFETY_H */
