/*
 * controller.h —— Controller 插件（§10.2，冻结）
 * 多速率：step_slow(1kHz) / step_fast(20kHz)；生命周期：on_enter/on_exit。
 * PID 增益经 ConfigSnapshot 传入 step_*（§10.2 收口）。
 * 静态绑定：第一版用 const ControllerOps，禁 Factory/Registry（§21）。
 */
#ifndef FOC_CONTROL_CONTROLLER_H
#define FOC_CONTROL_CONTROLLER_H

#include <stdint.h>
#include "foc/config.h"
#include "runtime/command.h"
#include "runtime/feedback.h"

#ifdef __cplusplus
extern "C" {
#endif

/* V0.1 控制链（§10.2.1）：voltage_sp = ControlSetpoint.voltage_q；current_q 为 V0.2 预留，V0.1 不赋值 */
typedef struct {
    float voltage_q, voltage_d;   /* [V] V0.1（voltage_sp） */
    float current_q;              /* [A] 预留 V0.2 */
    float torque;                 /* [Nm] 预留 V0.3 */
    uint32_t seq;
} ControlSetpoint;               /* slow → fast */

typedef struct {
    float voltage_d, voltage_q;   /* [V] V0.1 */
    float current_q;              /* [A] 预留 V0.2 */
    float torque;                 /* [Nm] 预留 V0.3 */
} ControlOutput;                 /* fast 输出 */

typedef struct {
    int (*init)(void *ctx);
    int (*reset)(void *ctx);
    int (*on_enter)(void *ctx, ControlMode mode);   /* 进入模式：如清零积分 */
    int (*on_exit)(void *ctx, ControlMode mode);    /* 离开模式 */
    int (*set_param)(void *ctx, uint32_t param_id, const void *val);  /* 调试期局部覆盖；运行期以快照 cfg 为准 */
    int (*step_slow)(void *ctx, const MotorCommand *cmd, const FastFeedback *fb,
                     const MotorRuntimeConfig *cfg, float dt_slow, ControlSetpoint *sp);
    int (*step_fast)(void *ctx, const FastFeedback *fb, const ControlSetpoint *sp,
                     const MotorRuntimeConfig *cfg, float dt_fast, ControlOutput *out);
} ControllerOps;

typedef struct { const ControllerOps *ops; void *ctx; } Controller;

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_CONTROLLER_H */
