/*
 * telemetry.h —— 遥测（§4.3，冻结）
 * ~100Hz 慢数据（温度/母线电压/fault），不阻塞 FOC。
 * 遥测数据在 Slow Task / Service 快照后输出，禁止 fast_step 内 printf（§12）。
 */
#ifndef FOC_RUNTIME_TELEMETRY_H
#define FOC_RUNTIME_TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    temperature_c;   /* [°C] */
    float    vbus_v;          /* [V] */
    float    velocity_radps;  /* [rad/s] */
    float    angle_rad;       /* [rad] */
    uint32_t fault_current;   /* 位图 */
    uint32_t fault_latched;   /* 位图 */
    uint64_t timestamp_us;
} Telemetry;

void telemetry_init(Telemetry *t);

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_TELEMETRY_H */
