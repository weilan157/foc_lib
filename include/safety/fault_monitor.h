/*
 * fault_monitor.h —— FaultMonitor（§16.5，必修 7，冻结）
 * 统一故障判定入口：debounce / hysteresis；禁止散落 if(temp>90) fault()。
 */
#ifndef FOC_SAFETY_FAULT_MONITOR_H
#define FOC_SAFETY_FAULT_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "safety/fault.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FaultCode      code;
    FaultSeverity  severity;
    uint32_t       debounce_count;    /* 连续 N 次超限才报 */
    uint32_t       debounce_thresh;
    float          hi_thresh;         /* 滞回上限 */
    float          lo_thresh;         /* 滞回下限（低于此才清除） */
    bool           active;            /* 当前是否处于超限态 */
} FaultMonitorChannel;

typedef struct {
    FaultMonitorChannel chan[FAULT_CODE_COUNT];
    uint32_t            run_count;
} FaultMonitor;

void fault_monitor_init(FaultMonitor *fm);
int  fault_monitor_update(FaultMonitor *fm, FaultCode code, float value, FaultReg *fault);
/* 内部：debounce（连续计数）→ hysteresis（hi/lo）→ fault_raise / fault_clear */
void fault_monitor_configure(FaultMonitor *fm, FaultCode code, FaultSeverity sev,
                             float hi, float lo, uint32_t debounce_thresh);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SAFETY_FAULT_MONITOR_H */
