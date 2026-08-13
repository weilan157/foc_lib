/*
 * motor_runtime.h —— MotorRuntime（§10.3 / §4.4，冻结）
 * V0.1.6：经 HardwareAdapter，不含硬件指针；无 RuntimeConfig 指针（只经 ConfigSnapshot）。
 * Feedback 唯一出口：FeedbackBuffer（禁 angle/velocity 副本）。
 */
#ifndef FOC_CORE_MOTOR_RUNTIME_H
#define FOC_CORE_MOTOR_RUNTIME_H

#include <stdint.h>
#include "foc/config.h"
#include "control/controller.h"
#include "control/calibration.h"
#include "device/hw_adapter.h"
#include "runtime/command.h"
#include "runtime/setpoint.h"
#include "runtime/feedback.h"
#include "runtime/config_snapshot.h"
#include "runtime/stats.h"
#include "runtime/telemetry.h"
#include "safety/fault.h"
#include "safety/fault_monitor.h"
#include "safety/safety.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 生命周期状态机（§9.1，冻结） */
typedef enum {
    FOC_STATE_CREATED = 0,
    FOC_STATE_INIT,
    FOC_STATE_SELF_TEST,
    FOC_STATE_CALIBRATION,
    FOC_STATE_READY,
    FOC_STATE_RUNNING,
    FOC_STATE_STOPPING,
    FOC_STATE_FAULT,
    FOC_STATE_LOCK
} FocState;

typedef struct {
    uint32_t           idx;
    const MotorStaticConfig *scfg;   /* 只读：硬件属性 + cap + 默认 limits（无 RuntimeConfig 指针） */
    MotorCalibration   calib;        /* 只读：校准自动产生 */

    FocState           state;
    FaultReg          *fault;
    uint64_t           timestamp;
    RuntimeStats       stats;

    Controller         controller;
    CalibrationOps     calibration;
    void              *cal_ctx;        /* 校准插件上下文（board 注入，§9.2） */
    Safety             safety;         /* 独立安全层（§15） */

    CommandBuffer      cmd_buf;        /* 命令唯一来源 */
    SetpointBuffer     sp_buf;         /* Slow → Fast */
    FeedbackBuffer     fb_buf;         /* 反馈唯一出口（必修 6） */
    ConfigSnapshot     cfg_snapshot;   /* RuntimeConfig 唯一访问途径（必修 1/4） */
    HardwareAdapter    hw;             /* 唯一硬件入口（真实或仿真） */
    Telemetry         *tel;
    FaultMonitor       fault_monitor;  /* §16.5 每电机一份 */
} MotorRuntime;

/* 生命周期 API（§9.1/§9.3，冻结） */
int motor_init(MotorRuntime *rt, uint32_t idx, const MotorStaticConfig *scfg,
               const HwAdapterOps *hw_ops, void *hw_ctx,
               const ControllerOps *ctrl_ops, void *ctrl_ctx,
               const CalibrationOps *cal_ops, void *cal_ctx);
int motor_load_config(MotorRuntime *rt, const MotorRuntimeConfig *rcfg);
int motor_self_test(MotorRuntime *rt);
int motor_calibrate(MotorRuntime *rt);
int motor_prepare(MotorRuntime *rt);
int motor_enable(MotorRuntime *rt);
int motor_stop(MotorRuntime *rt);
int motor_recover(MotorRuntime *rt);

/* 多速率执行（§4.4，冻结）：TimeBase 单一时间源（方案 B） */
void motor_slow_step(MotorRuntime *rt, const TimeBase *tb);   /* 1kHz：位置/速度环 */
void motor_fast_step(MotorRuntime *rt, const TimeBase *tb);   /* 20kHz：电压环 + FOC */

/* 执行预算检查（§4.8.1 硬约束）：board 的 FOC Task 测完 fast_step 执行时间后调用；
   记账 max_exec_us / overrun_count，超预算 → FAULT_CONTROL_OVERRUN
   （防止高优先级 fast loop 饿死低优先级 Service/CLI 任务——串口不响应的典型根因） */
void motor_fast_loop_budget(MotorRuntime *rt, uint32_t exec_us, uint32_t budget_us);

/* 安全关断唯一出口（§16.1，冻结）：FaultManager 禁止直接操作 PWM */
void motor_enter_safe_state(MotorRuntime *rt, FaultCode code);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CORE_MOTOR_RUNTIME_H */
