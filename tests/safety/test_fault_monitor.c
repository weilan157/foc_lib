/*
 * test_fault_monitor.c —— FaultMonitor（§16.5，必修 7）：debounce + hysteresis
 */
#include "safety/fault_monitor.h"
#include "../test_assert.h"

int main(void)
{
    FaultMonitor fm;
    FaultReg fault;
    int i;

    fault_init(&fault);
    fault_monitor_init(&fm);
    fault_monitor_configure(&fm, FAULT_OVERTEMP, SEV_WARN, 90.0f, 85.0f, 3u);

    /* debounce：连续 3 次超限才报 */
    for (i = 0; i < 2; i++) {
        (void)fault_monitor_update(&fm, FAULT_OVERTEMP, 91.0f, &fault);
        CHECK(!fault_is_set(&fault, FAULT_OVERTEMP));   /* 未达阈值 */
    }
    (void)fault_monitor_update(&fm, FAULT_OVERTEMP, 91.0f, &fault);
    CHECK(fault_is_set(&fault, FAULT_OVERTEMP));        /* 第 3 次触发 */

    /* hysteresis：降到 lo 以下才清除 */
    (void)fault_monitor_update(&fm, FAULT_OVERTEMP, 88.0f, &fault);   /* 滞回带内：保持 */
    CHECK(fault_is_set(&fault, FAULT_OVERTEMP));
    (void)fault_monitor_update(&fm, FAULT_OVERTEMP, 84.0f, &fault);   /* 低于 lo：清除 */
    CHECK(!fault_is_set(&fault, FAULT_OVERTEMP));

    TEST_REPORT();
}
