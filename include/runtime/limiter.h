/*
 * limiter.h —— 输出限幅（§14，冻结）
 * 输出 Limiter 在 motor_fast_step()：limiter_apply(&out, &rt->scfg->cap, &cfg)
 *   - 能力包络取 scfg->cap（静态）
 *   - 限幅表取 ConfigSnapshot 内 cfg.limits（运行期）
 * Controller 不感知具体规格。
 */
#ifndef FOC_RUNTIME_LIMITER_H
#define FOC_RUNTIME_LIMITER_H

#include "foc/config.h"
#include "control/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

void limiter_apply(ControlOutput *out, const MotorCapability *cap, const MotorRuntimeConfig *rcfg);

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_LIMITER_H */
