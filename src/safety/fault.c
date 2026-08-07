/*
 * fault.c —— Fault 位图 + FaultAction 静态表（§16.1/§16.2，冻结）
 * FaultManager 只 raise/clear/classify；唯一有权 pwm_disable 的是 motor_enter_safe_state()。
 */
#include "safety/fault.h"

void fault_init(FaultReg *f)
{
    if (f == NULL) { return; }
    f->latched = 0u;
    f->current = 0u;
}

void fault_raise(FaultReg *f, FaultCode c, FaultSeverity sev)
{
    uint32_t bit;

    if (f == NULL || c == FAULT_NONE || c >= FAULT_CODE_COUNT) { return; }
    bit = (uint32_t)1u << (uint32_t)c;

    f->current |= bit;
    /* CRITICAL 或 LATCH 类故障才锁存（FAULT_SAFETY/OVERTEMP_HARD/SELF_TEST 等） */
    if (sev == SEV_CRITICAL) { f->latched |= bit; }
}

void fault_clear(FaultReg *f, FaultCode c)
{
    uint32_t bit;

    if (f == NULL || c == FAULT_NONE || c >= FAULT_CODE_COUNT) { return; }
    bit = (uint32_t)1u << (uint32_t)c;

    f->current &= ~bit;
    f->latched &= ~bit;
}

bool fault_is_set(const FaultReg *f, FaultCode c)
{
    if (f == NULL || c == FAULT_NONE || c >= FAULT_CODE_COUNT) { return false; }
    return ((f->current & ((uint32_t)1u << (uint32_t)c)) != 0u);
}

FaultAction fault_get_action(FaultCode c)
{
    static const FaultAction table[FAULT_CODE_COUNT] = {
        [FAULT_NONE]            = FAULT_ACTION_RETRY,
        [FAULT_OVERCURRENT]     = FAULT_ACTION_SHUTDOWN,
        [FAULT_OVERVOLTAGE]     = FAULT_ACTION_SHUTDOWN,
        [FAULT_UNDERVOLTAGE]    = FAULT_ACTION_RETRY,
        [FAULT_OVERTEMP]        = FAULT_ACTION_RETRY,
        [FAULT_OVERTEMP_HARD]   = FAULT_ACTION_LATCH,
        [FAULT_OVERVELOCITY]    = FAULT_ACTION_SHUTDOWN,
        [FAULT_ENCODER_LOSS]    = FAULT_ACTION_SHUTDOWN,
        [FAULT_ENCODER_QUALITY] = FAULT_ACTION_SHUTDOWN,
        [FAULT_TIMING]          = FAULT_ACTION_SHUTDOWN,
        [FAULT_CALIBRATION]     = FAULT_ACTION_SHUTDOWN,
        [FAULT_SELF_TEST]       = FAULT_ACTION_LATCH,
        [FAULT_GATE_DRIVER]     = FAULT_ACTION_SHUTDOWN,
        [FAULT_COMM_TIMEOUT]    = FAULT_ACTION_RETRY,
        [FAULT_CONTROL_OVERRUN] = FAULT_ACTION_SHUTDOWN,
        [FAULT_SAFETY]          = FAULT_ACTION_LATCH,
    };

    if (c >= FAULT_CODE_COUNT) { return FAULT_ACTION_RETRY; }
    return table[c];
}
