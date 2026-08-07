/*
 * safety.c —— 独立 Safety 层（§15，冻结）
 * 安全逻辑不依赖控制算法；WDT 超时 / 急停 → 直接安全关断（不经控制链）。
 */
#include "safety/safety.h"

void safety_init(Safety *s)
{
    if (s == NULL) { return; }
    s->state          = SAFE_OK;
    s->fault          = NULL;   /* 由 board / 组装者注入；必须先初始化，防垃圾指针 */
    s->hw             = NULL;   /* 同上：board 注入 HalSafetyOps 前必须为 NULL */
    s->last_feed_us   = 0u;
    s->wdt_timeout_us = 0u;     /* 0 = 未配置超时（由 board 配置） */
}

void safety_enter_sto(Safety *s)
{
    if (s == NULL) { return; }
    s->state = SAFE_STO;
    if (s->hw != NULL && s->hw->hal_set_sto != NULL) { s->hw->hal_set_sto(true); }
    if (s->fault != NULL) { fault_raise(s->fault, FAULT_SAFETY, SEV_CRITICAL); }
}

void safety_enter_estop(Safety *s)
{
    if (s == NULL) { return; }
    s->state = SAFE_ESTOP;
    if (s->hw != NULL && s->hw->hal_set_estop != NULL) { s->hw->hal_set_estop(true); }
    if (s->fault != NULL) { fault_raise(s->fault, FAULT_SAFETY, SEV_CRITICAL); }
}

void safety_watchdog_feed(Safety *s, uint64_t now_us)
{
    if (s == NULL) { return; }
    s->last_feed_us = now_us;
    if (s->hw != NULL && s->hw->hal_wdt_feed != NULL) { s->hw->hal_wdt_feed(); }
}

void safety_check(Safety *s, uint64_t now_us)
{
    if (s == NULL) { return; }

    /* WDT 超时：喂狗超时（wdt_timeout_us != 0 才检查） */
    if ((s->wdt_timeout_us != 0u) && (now_us > s->last_feed_us) &&
        ((now_us - s->last_feed_us) > s->wdt_timeout_us)) {
        s->state = SAFE_FAULT_LATCH;
        if (s->hw != NULL && s->hw->hal_set_estop != NULL) { s->hw->hal_set_estop(true); }
        if (s->fault != NULL) { fault_raise(s->fault, FAULT_SAFETY, SEV_CRITICAL); }
        return;
    }
}
