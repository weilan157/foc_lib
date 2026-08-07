/*
 * limiter.c —— 输出 Limiter（§14，冻结）
 * 依据：MotorStaticConfig.cap（能力包络，静态）+ ConfigSnapshot 内 cfg.limits（运行期按模式）。
 * Controller 不感知具体规格。
 */
#include "runtime/limiter.h"
#include "foc_types.h"

void limiter_apply(ControlOutput *out, const MotorCapability *cap, const MotorRuntimeConfig *rcfg)
{
    float max_volt;

    if (out == NULL) { return; }

    /* V0.1：只限电压（voltage_sp）；电流/力矩为 V0.2/V0.3 预留 */
    max_volt = (cap != NULL) ? cap->max_voltage : 0.0f;

    if (out->voltage_q >  max_volt) { out->voltage_q =  max_volt; }
    if (out->voltage_q < -max_volt) { out->voltage_q = -max_volt; }
    if (out->voltage_d >  max_volt) { out->voltage_d =  max_volt; }
    if (out->voltage_d < -max_volt) { out->voltage_d = -max_volt; }

    (void)rcfg;   /* V0.1：输出限幅仅用 cap；按模式表在输入 Limiter（motor_slow_step） */
}
