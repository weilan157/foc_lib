/*
 * gate_driver.h —— GateDriver 功率级管理器（§7.1，冻结）
 * 分级：gate_fast_check（20kHz 位读取）/ gate_status_update（~100Hz 慢速）。
 * VoltageVector 唯一归属 foc_math.h（§3.3），此处引用，禁止重复定义。
 */
#ifndef FOC_DEVICE_GATE_DRIVER_H
#define FOC_DEVICE_GATE_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "foc/foc_math.h"   /* VoltageVector{alpha_v, beta_v}（foc_inverse_park 输出类型） */

#ifdef __cplusplus
extern "C" {
#endif

/* FastFault —— 硬件快速故障标志（20kHz 直接读引脚/寄存器位，不做解析） */
typedef struct {
    uint32_t fault_flags;   /* bit0 EN_fault / bit1 comparator / bit2 TIM_break …（板定义） */
} GateFastFault;

/* SlowStatus：温度 / VBUS / 寄存器（~100Hz） */
typedef struct {
    uint32_t fault_code;    /* OCP/UVLO/OTSD… */
    uint16_t status;
    float    temperature;   /* [°C] */
    float    vbus;          /* [V] */
} GateDriverStatus;

typedef struct {
    int (*enable)(void *ctx);
    int (*disable)(void *ctx);
    int (*set_output)(void *ctx, const VoltageVector *v);
    int (*gate_fast_check)(void *ctx, GateFastFault *fault);     /* 20kHz：位读取，无解析 */
    int (*gate_status_update)(void *ctx, GateDriverStatus *st);  /* ~100Hz：温度 / VBUS / 寄存器 */
} GateDriverOps;

typedef struct { const GateDriverOps *ops; void *ctx; } GateDriver;

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_GATE_DRIVER_H */
