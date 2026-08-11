/*
 * test_current_loop.c —— 电流环端到端闭环（R-L 电机模型 + 电流 PI 跟踪）
 *
 * 验证 V0.2 电流环真实控制电流：iq_sp → current_controller.step_fast → vq
 * → R-L 模型（diq/dt=(vq−R·iq)/L，we=0 简化）→ iq 反馈 → 收敛到 iq_sp。
 * 电角度固定 π/2，反馈电流由模型 iq 反算 ia/ib（Park 逆）。
 */
#include "control/current_controller.h"
#include "../test_assert.h"
#include <string.h>

#define PI (3.141592653589793f)

int main(void)
{
    CurrentControllerCtx cctx = {0};
    MotorRuntimeConfig cfg;
    FastFeedback fb;
    ControlSetpoint sp;
    ControlOutput out;
    uint32_t i;

    /* 电机模型：R=1Ω, L=1mH（增益由带宽推导） */
    const float R = 1.0f;
    const float L = 0.001f;
    const float dt = 0.00005f;      /* 20kHz */
    const float iq_sp = 2.0f;       /* 电流目标 [A] */
    float iq = 0.0f;                /* 模型 q 轴电流 */

    memset(&cfg, 0, sizeof(cfg));
    cfg.kp = 1.0f;
    cfg.ki = 0.0f;
    cfg.limits.limit[CTRL_MODE_TORQUE].max = 10.0f;
    cfg.limits.limit[CTRL_MODE_TORQUE].min = -10.0f;
    cfg.current_kp = 0.0f;          /* 走带宽推导 */
    cfg.current_ki = 0.0f;
    cfg.current_bandwidth_hz = 1000.0f;   /* kp=1, ki=1000 */
    cfg.current_filter_hz = 0.0f;

    current_controller_init_ctx(&cctx);
    cctx.max_current = 5.0f;
    cctx.pole_pairs = 4u;
    cctx.phase_resistance = R;
    cctx.phase_inductance = L;
    cctx.flux_linkage = 0.01f;
    cctx.v_limit = 24.0f;

    /* 设定值：iq_sp */
    sp.current_q = iq_sp;
    sp.seq = 1u;

    /* 闭环：电流 PI → 模型 → 反馈 */
    for (i = 0u; i < 2000u; i++) {
        /* 反馈：Park 逆（电角度 π/2）：id=beta, iq=-alpha → alpha=-iq, beta=0 */
        fb.elec_angle_rad = PI / 2.0f;
        fb.mech_vel_radps = 0.0f;                     /* we=0：无 BEMF/解耦 */
        fb.ia = -iq;
        fb.ib =  iq / 2.0f;

        (void)current_controller_ops.step_fast(&cctx, &fb, &sp, &cfg, dt, &out);

        /* R-L 模型：diq/dt = (vq − R·iq)/L */
        iq += dt * (out.voltage_q - R * iq) / L;
    }

    /* 收敛：iq → iq_sp（容差 ±0.05A） */
    CHECK_NEAR(iq, iq_sp, 0.05f);
    CHECK_NEAR(cctx.last_iq_measured, iq_sp, 0.05f);

    /* 稳态电压 ≈ R·iq（电感压降为 0）：vq ≈ R·iq_sp = 2V */
    CHECK_NEAR(out.voltage_q, R * iq_sp, 0.1f);

    /* 反向目标也收敛 */
    sp.current_q = -1.0f;
    for (i = 0u; i < 1500u; i++) {
        fb.elec_angle_rad = PI / 2.0f;
        fb.mech_vel_radps = 0.0f;
        fb.ia = -iq;
        fb.ib = iq / 2.0f;
        (void)current_controller_ops.step_fast(&cctx, &fb, &sp, &cfg, dt, &out);
        iq += dt * (out.voltage_q - R * iq) / L;
    }
    CHECK_NEAR(iq, -1.0f, 0.05f);

    TEST_REPORT();
}
