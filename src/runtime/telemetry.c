/*
 * telemetry.c —— 遥测（§4.3，冻结）
 * ~100Hz 慢数据，不阻塞 FOC；遥测在 Slow Task / Service 快照后输出。
 */
#include "runtime/telemetry.h"

void telemetry_init(Telemetry *t)
{
    if (t == NULL) { return; }
    t->temperature_c = 0.0f;
    t->vbus_v        = 0.0f;
    t->velocity_radps = 0.0f;
    t->angle_rad     = 0.0f;
    t->fault_current = 0u;
    t->fault_latched = 0u;
    t->timestamp_us  = 0u;
}
