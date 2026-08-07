/*
 * test_pid_controller.c —— V0.1 电压模式级联控制器（§10.2.1）
 * 验证：voltage_sp 语义（禁 current_q）；TORQUE=电压直通；Position→Velocity→Voltage。
 */
#include "control/pid_controller.h"
#include "../test_assert.h"

int main(void)
{
    PidControllerCtx pctx;
    Controller ctrl = { .ops = &pid_controller_ops, .ctx = &pctx };
    MotorRuntimeConfig cfg = { .kp = 0.5f, .ki = 0.2f, .kd = 0.0f };
    FastFeedback fb = { .mech_angle_rad = 0.0f, .mech_vel_radps = 0.0f, .elec_angle_rad = 0.0f, .quality = FEEDBACK_OK };
    ControlSetpoint sp = { 0 };
    ControlOutput out;

    CHECK(ctrl.ops->init(ctrl.ctx) == 0);

    /* VELOCITY：目标 5 rad/s，当前 0 → voltage_q > 0 */
    {
        MotorCommand cmd = { .target = 5.0f, .mode = CTRL_MODE_VELOCITY, .sequence = 1u };
        CHECK(ctrl.ops->on_enter(ctrl.ctx, CTRL_MODE_VELOCITY) == 0);
        CHECK(ctrl.ops->step_slow(ctrl.ctx, &cmd, &fb, &cfg, 0.001f, &sp) == 0);
        CHECK(sp.voltage_q > 0.0f);          /* voltage_sp 正 */
        CHECK(sp.voltage_d == 0.0f);
        CHECK(sp.current_q == 0.0f);         /* V0.1 禁 iq_sp */
        CHECK(sp.seq == 1u);
    }

    /* TORQUE：voltage_q = cmd.target（电压直通，无电流环） */
    {
        MotorCommand cmd = { .target = 3.0f, .mode = CTRL_MODE_TORQUE, .sequence = 2u };
        CHECK(ctrl.ops->step_slow(ctrl.ctx, &cmd, &fb, &cfg, 0.001f, &sp) == 0);
        CHECK_NEAR(sp.voltage_q, 3.0f, 1e-6f);
        CHECK(sp.current_q == 0.0f);
    }

    /* step_fast：V0.1 电压直通 */
    {
        ControlSetpoint sp2 = { .voltage_q = 4.0f, .voltage_d = 1.0f, .seq = 3u };
        CHECK(ctrl.ops->step_fast(ctrl.ctx, &fb, &sp2, &cfg, 0.00005f, &out) == 0);
        CHECK_NEAR(out.voltage_q, 4.0f, 1e-6f);
        CHECK_NEAR(out.voltage_d, 1.0f, 1e-6f);
        CHECK(out.current_q == 0.0f);
    }

    /* 模式切换：on_exit 清积分 */
    {
        MotorCommand cmd = { .target = 5.0f, .mode = CTRL_MODE_VELOCITY, .sequence = 4u };
        (void)ctrl.ops->step_slow(ctrl.ctx, &cmd, &fb, &cfg, 0.001f, &sp);
        CHECK(pctx.integral != 0.0f);
        CHECK(ctrl.ops->on_exit(ctrl.ctx, CTRL_MODE_VELOCITY) == 0);
        CHECK(pctx.integral == 0.0f);        /* 切换清零积分 */
    }

    TEST_REPORT();
}
