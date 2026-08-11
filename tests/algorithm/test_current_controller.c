/*
 * test_current_controller.c —— V0.2 电流环级联控制器单测
 *
 * 覆盖：带宽自动推导、Slow 环（速度/位置/TORQUE → iq_sp）、iq_sp 限幅、
 *       Fast 电流 PI（误差→电压、BEMF 前馈、交叉解耦）、非法入参。
 */
#include "control/current_controller.h"
#include "../test_assert.h"
#include <string.h>

#define PI (3.141592653589793f)

static void rig_cfg(MotorRuntimeConfig *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->kp = 2.0f;
    cfg->ki = 0.0f;
    cfg->kd = 0.0f;
    cfg->limits.limit[CTRL_MODE_TORQUE].max = 10.0f;
    cfg->limits.limit[CTRL_MODE_TORQUE].min = -10.0f;
    cfg->current_kp = 2.0f;
    cfg->current_ki = 10.0f;
    cfg->current_bandwidth_hz = 0.0f;
    cfg->current_filter_hz = 0.0f;
}

int main(void)
{
    CurrentControllerCtx cctx = {0};
    MotorRuntimeConfig cfg;
    MotorCommand cmd;
    FastFeedback fb;
    ControlSetpoint sp;
    ControlOutput out;
    float kp = 0.0f, ki = 0.0f;

    current_controller_init_ctx(&cctx);
    cctx.max_current = 5.0f;
    cctx.pole_pairs = 4u;
    cctx.phase_resistance = 1.0f;
    cctx.phase_inductance = 0.001f;
    cctx.flux_linkage = 0.01f;

    /* ① 带宽自动推导（ODrive 式）：kp=bw·L, ki=bw·R */
    current_controller_derive_gains(1000.0f, 1.0f, 0.001f, &kp, &ki);
    CHECK_NEAR(kp, 1.0f, 1e-4f);
    CHECK_NEAR(ki, 1000.0f, 1e-3f);

    /* ② Slow 环：VELOCITY → iq_sp（速度 PI，积分 ki=0） */
    rig_cfg(&cfg);
    cmd.mode = CTRL_MODE_VELOCITY;
    cmd.target = 5.0f;
    fb.mech_angle_rad = 0.0f;
    fb.mech_vel_radps = 3.0f;
    fb.elec_angle_rad = 0.0f;
    CHECK(current_controller_ops.step_slow(&cctx, &cmd, &fb, &cfg, 0.001f, &sp) == 0);
    CHECK_NEAR(sp.current_q, 2.0f * (5.0f - 3.0f), 1e-3f);   /* 4.0 */

    /* ③ Slow 环：TORQUE → iq_sp = 命令（限幅内） */
    cmd.mode = CTRL_MODE_TORQUE;
    cmd.target = 2.0f;
    CHECK(current_controller_ops.step_slow(&cctx, &cmd, &fb, &cfg, 0.001f, &sp) == 0);
    CHECK_NEAR(sp.current_q, 2.0f, 1e-4f);

    /* ④ iq_sp 限幅：能力上限 max_current=5（命令 8 → 5） */
    cmd.target = 8.0f;
    CHECK(current_controller_ops.step_slow(&cctx, &cmd, &fb, &cfg, 0.001f, &sp) == 0);
    CHECK_NEAR(sp.current_q, 5.0f, 1e-4f);

    /* ⑤ Slow 环：POSITION → iq_sp（位置 P 经速度 PI） */
    cmd.mode = CTRL_MODE_POSITION;
    cmd.target = 1.0f;
    fb.mech_angle_rad = 0.5f;
    fb.mech_vel_radps = 0.0f;
    cctx.integral = 0.0f;
    CHECK(current_controller_ops.step_slow(&cctx, &cmd, &fb, &cfg, 0.001f, &sp) == 0);
    CHECK(sp.current_q > 0.0f);                                 /* 位置误差 → 正向电流 */

    /* ⑥ Fast 环：电流 PI（elec=π/2, ia=-1, ib=0.5 → iq=1, id=0；iq_sp=1 → ierr=0） */
    fb.elec_angle_rad = PI / 2.0f;
    fb.ia = -1.0f;
    fb.ib = 0.5f;
    fb.mech_vel_radps = 0.0f;                                    /* we=0 → 前馈/解耦=0 */
    sp.current_q = 1.0f;
    cctx.vq_integral = 0.0f;
    cctx.vd_integral = 0.0f;
    CHECK(current_controller_ops.step_fast(&cctx, &fb, &sp, &cfg, 0.00005f, &out) == 0);
    CHECK_NEAR(out.voltage_q, 0.0f, 1e-3f);                     /* ierr=0，无前馈 */

    /* ⑦ Fast 环：iq 反馈 0.5 → ierr=0.5 → vq = 0.5·kp(=2) = 1.0 */
    fb.ia = -0.5f;
    fb.ib = 0.25f;                                               /* iq=0.5 */
    cctx.vq_integral = 0.0f;
    cctx.vd_integral = 0.0f;
    CHECK(current_controller_ops.step_fast(&cctx, &fb, &sp, &cfg, 0.00005f, &out) == 0);
    CHECK_NEAR(out.voltage_q, 1.0f, 1e-2f);

    /* ⑧ BEMF 前馈：we=40 → ff_q = flux·we = 0.4（ierr=0 时 vq≈0.4） */
    fb.ia = -1.0f;
    fb.ib = 0.5f;                                                /* iq=1 = iq_sp → ierr=0 */
    fb.mech_vel_radps = 10.0f;                                   /* we=40 */
    cctx.vq_integral = 0.0f;
    cctx.vd_integral = 0.0f;
    CHECK(current_controller_ops.step_fast(&cctx, &fb, &sp, &cfg, 0.00005f, &out) == 0);
    CHECK_NEAR(out.voltage_q, 0.4f, 1e-2f);                     /* BEMF 前馈 */

    /* ⑨ 交叉解耦：vd = dec_d = -we·L·iq_sp = -40·0.001·1 = -0.04 */
    CHECK_NEAR(out.voltage_d, -0.04f, 1e-2f);

    /* ⑩ 带宽推导路径：cfg.current_kp/ki=0 + bandwidth → 推导 */
    rig_cfg(&cfg);
    cfg.current_kp = 0.0f;
    cfg.current_ki = 0.0f;
    cfg.current_bandwidth_hz = 1000.0f;                         /* kp=1, ki=1000 */
    fb.ia = -0.5f; fb.ib = 0.25f; fb.elec_angle_rad = PI / 2.0f;
    fb.mech_vel_radps = 0.0f;
    sp.current_q = 1.0f;
    cctx.vq_integral = 0.0f;
    cctx.vd_integral = 0.0f;
    CHECK(current_controller_ops.step_fast(&cctx, &fb, &sp, &cfg, 0.00005f, &out) == 0);
    CHECK_NEAR(out.voltage_q, 0.5f, 1e-2f);                     /* 0.5·kp(=1) */

    /* ⑪ on_enter/on_exit/reset 清积分 */
    CHECK(current_controller_ops.on_enter(&cctx, CTRL_MODE_POSITION) == 0);
    CHECK(cctx.active_mode == CTRL_MODE_POSITION);
    CHECK(cctx.integral == 0.0f);
    CHECK(cctx.vq_integral == 0.0f);

    /* ⑫ 非法入参 */
    CHECK(current_controller_ops.step_slow(NULL, &cmd, &fb, &cfg, 0.001f, &sp) != 0);
    CHECK(current_controller_ops.step_fast(&cctx, NULL, &sp, &cfg, 0.00005f, &out) != 0);
    CHECK(current_controller_ops.set_param(&cctx, 99u, &kp) != 0);

    TEST_REPORT();
}
