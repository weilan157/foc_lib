/*
 * hw_adapter.h —— HardwareAdapter（§7.4，冻结）
 * MotorControl 唯一硬件入口（真实板 或 仿真 motor_model）。
 * Motor 不直接持有 GateDriver / PositionSensor / CurrentSense 指针。
 */
#ifndef FOC_DEVICE_HW_ADAPTER_H
#define FOC_DEVICE_HW_ADAPTER_H

#include "device/position_sensor.h"
#include "device/gate_driver.h"
#include "runtime/sampling.h"   /* SampleFrame */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int (*sensor_update)(void *ctx);
    int (*sensor_get_feedback)(void *ctx, EncoderFeedback *fb);
    int (*current_reconstruct)(void *ctx, SampleFrame *sf);      /* V0.2 */
    int (*gate_set_output)(void *ctx, const VoltageVector *v);
    int (*gate_fast_check)(void *ctx, GateFastFault *fault);     /* 20kHz 硬件快速故障 */
    int (*gate_status_update)(void *ctx, GateDriverStatus *st);  /* ~100Hz 慢速状态 */
} HwAdapterOps;

typedef struct {
    const HwAdapterOps *ops;
    void               *ctx;    /* 真实：聚合 GateDriver/PositionSensor/CurrentSense；仿真：motor_model */
} HardwareAdapter;

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_HW_ADAPTER_H */
