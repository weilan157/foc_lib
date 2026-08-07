/*
 * hw_adapter_sim.h —— 仿真 HardwareAdapter（§7.4 / §25，冻结）
 * 提供 HwAdapterOps：电压 → motor_model → 位置/速度反馈，FOC 在 PC 闭环跑。
 */
#ifndef FOC_TEST_SIMULATION_HW_ADAPTER_SIM_H
#define FOC_TEST_SIMULATION_HW_ADAPTER_SIM_H

#include "device/hw_adapter.h"
#include "simulation/motor_model.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    MotorModel        *model;
    float              sim_dt;        /* 仿真步长 [s]（默认 1/20000） */
    uint32_t           fast_fault_flags;
    GateDriverStatus   status;
    VoltageVector      last_vv;
    uint32_t           set_output_calls;
} SimHwCtx;

/* 全局常量 ops（各实例共用）；ctx 持有模型指针 */
extern const HwAdapterOps hw_adapter_sim_ops;

void sim_hw_init(SimHwCtx *ctx, MotorModel *model);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TEST_SIMULATION_HW_ADAPTER_SIM_H */
