/*
 * pid_controller.h —— V0.1 电压模式级联控制器（§10.2.1，冻结语义）
 * 控制链：Position P → Velocity PI → Voltage Command(voltage_sp) → Voltage FOC → SVPWM
 * 变量必须 voltage_sp（ControlSetpoint.voltage_q）；禁止 iq_sp/current_q 参与 V0.1。
 * step_fast（20kHz）：V0.1 电压直通（无电流环），V0.2 在此插入 Current PI。
 */
#ifndef FOC_CONTROL_PID_CONTROLLER_H
#define FOC_CONTROL_PID_CONTROLLER_H

#include <stdint.h>
#include "control/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 速度 PI（Slow 环，1kHz）：运行期增益取自 ConfigSnapshot 内 cfg.kp/ki/kd */
    float   integral;            /* 积分累加 */
    float   integral_limit;      /* 防 windup */
    /* 位置 P（Slow 环，1kHz）：V0.1 增益在 ctx（V0.2 再入参数体系） */
    float   kp_pos;              /* 位置误差 → 速度目标 [1/s] */
    float   vel_limit_from_pos;  /* 位置环输出的速度目标限幅 [rad/s] */
    ControlMode active_mode;
    float   last_velocity_target;
} PidControllerCtx;

/* 静态绑定（§10.2 收口）：const ControllerOps，不做注册表 */
extern const ControllerOps pid_controller_ops;

void pid_controller_init_ctx(PidControllerCtx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_PID_CONTROLLER_H */
