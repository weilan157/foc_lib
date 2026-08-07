/*
 * fault_monitor.c —— FaultMonitor（§16.5，必修 7，冻结）
 * 统一故障判定：debounce（连续 N 次超限才报）+ hysteresis（低于 lo 才清除）。
 * 替代散落的 if(temp>90) fault()。
 */
#include "safety/fault_monitor.h"
#include "foc_types.h"

void fault_monitor_init(FaultMonitor *fm)
{
    uint32_t i;

    if (fm == NULL) { return; }
    for (i = 0u; i < FAULT_CODE_COUNT; i++) {
        fm->chan[i].code             = (FaultCode)i;
        fm->chan[i].severity         = SEV_CRITICAL;
        fm->chan[i].debounce_count   = 0u;
        fm->chan[i].debounce_thresh  = 1u;
        fm->chan[i].hi_thresh        = 0.0f;
        fm->chan[i].lo_thresh        = 0.0f;
        fm->chan[i].active           = false;
    }
    fm->run_count = 0u;
}

void fault_monitor_configure(FaultMonitor *fm, FaultCode code, FaultSeverity sev,
                             float hi, float lo, uint32_t debounce_thresh)
{
    if (fm == NULL || code >= FAULT_CODE_COUNT) { return; }
    fm->chan[code].severity        = sev;
    fm->chan[code].hi_thresh       = hi;
    fm->chan[code].lo_thresh       = lo;
    fm->chan[code].debounce_thresh = (debounce_thresh == 0u) ? 1u : debounce_thresh;
    fm->chan[code].debounce_count  = 0u;
    fm->chan[code].active          = false;
}

int fault_monitor_update(FaultMonitor *fm, FaultCode code, float value, FaultReg *fault)
{
    FaultMonitorChannel *ch;

    if (fm == NULL || code >= FAULT_CODE_COUNT || fault == NULL) { return FOC_ERROR; }
    ch = &fm->chan[code];

    if (value >= ch->hi_thresh) {
        /* 超上限：防抖计数 */
        ch->debounce_count++;
        if (!ch->active && (ch->debounce_count >= ch->debounce_thresh)) {
            ch->active = true;
            fault_raise(fault, code, ch->severity);
        }
    } else if (value <= ch->lo_thresh) {
        /* 低于下限：清除（滞回） */
        ch->debounce_count = 0u;
        if (ch->active) {
            ch->active = false;
            fault_clear(fault, code);
        }
    } else {
        /* 滞回带内：保持当前状态，清计数 */
        ch->debounce_count = 0u;
    }

    fm->run_count++;
    return FOC_OK;
}
