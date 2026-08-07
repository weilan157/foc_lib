/*
 * fault.h —— Fault 系统（§16.1/§16.2，冻结）
 * FaultManager 只 raise/clear/classify；唯一有权 pwm_disable/gate_disable 的是 motor_enter_safe_state()。
 */
#ifndef FOC_SAFETY_FAULT_H
#define FOC_SAFETY_FAULT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FAULT_NONE=0, FAULT_OVERCURRENT, FAULT_OVERVOLTAGE, FAULT_UNDERVOLTAGE,
    FAULT_OVERTEMP, FAULT_OVERTEMP_HARD, FAULT_OVERVELOCITY, FAULT_ENCODER_LOSS,
    FAULT_ENCODER_QUALITY, FAULT_TIMING, FAULT_CALIBRATION, FAULT_SELF_TEST,
    FAULT_GATE_DRIVER, FAULT_COMM_TIMEOUT, FAULT_CONTROL_OVERRUN, FAULT_SAFETY,
    FAULT_CODE_COUNT
} FaultCode;

typedef enum { SEV_INFO=0, SEV_WARN, SEV_CRITICAL } FaultSeverity;

/* latched/current 位图：bit = 1<<code */
typedef struct {
    uint32_t latched;
    uint32_t current;
} FaultReg;

typedef enum { FAULT_ACTION_RETRY, FAULT_ACTION_SHUTDOWN, FAULT_ACTION_LATCH } FaultAction;

void        fault_init(FaultReg *f);
void        fault_raise(FaultReg *f, FaultCode c, FaultSeverity sev);
void        fault_clear(FaultReg *f, FaultCode c);
bool        fault_is_set(const FaultReg *f, FaultCode c);
FaultAction fault_get_action(FaultCode c);   /* 静态 FaultAction 表（§16.2） */

#ifdef __cplusplus
}
#endif

#endif /* FOC_SAFETY_FAULT_H */
