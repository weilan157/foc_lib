/*
 * test_cal_abi_align.c —— ABZ 相位校准插件单测（注入 fake 电机）
 *
 * 覆盖：方向一致/反向的 inverted 判定、encoder_zero 反推（含编码器零偏、非零对齐角）、
 *       转子未跟随失败、非法入参。
 */
#include "control/cal_abi_align.h"
#include "../test_assert.h"

/* ---- fake 电机：转子跟随施加电角度；编码器可反、可带零偏 ---- */
typedef struct {
    float    theta_el;     /* 当前施加电角度 */
    float    vd, vq;
    uint32_t wait_total_ms;
    uint32_t pole_pairs;
    bool     enc_inverted; /* 模拟编码器方向反向 */
    float    mech_bias;    /* 编码器零点偏移 [rad] */
} FakeMotor;

static int fake_set_voltage(void *hw, float th, float vd, float vq)
{
    FakeMotor *m = (FakeMotor *)hw;
    m->theta_el = th; m->vd = vd; m->vq = vq;
    return 0;
}

static int fake_get_mech(void *hw, float *mech)
{
    FakeMotor *m = (FakeMotor *)hw;
    float mech_true = m->theta_el / (float)m->pole_pairs + m->mech_bias;  /* 转子跟随 + 零偏 */
    *mech = m->enc_inverted ? -mech_true : mech_true;
    return 0;
}

static int fake_wait(void *hw, uint32_t ms)
{
    ((FakeMotor *)hw)->wait_total_ms += ms;
    return 0;
}

int main(void)
{
    FakeMotor m = {0};
    CalAbiAlignCtx ctx = {0};

    /* ---- 场景1：方向一致 + 编码器零偏 → inverted=false，encoder_zero = -bias·pp ---- */
    m.pole_pairs = 4; m.enc_inverted = false; m.mech_bias = 0.2f;
    ctx.set_elec_voltage = fake_set_voltage;
    ctx.get_mech_angle = fake_get_mech;
    ctx.wait_ms = fake_wait;
    ctx.hw = &m;
    ctx.pole_pairs = 4;
    ctx.align_voltage = 1.5f;
    CHECK(cal_abi_align_phase(&ctx) == 0);
    CHECK(ctx.inverted == false);
    CHECK_NEAR(ctx.encoder_zero, -0.2f * 4.0f, 1e-4f);   /* -0.8 */
    CHECK(m.wait_total_ms > 0u);                          /* 确实走完阻塞流程 */

    /* ---- 场景2：编码器方向反 → inverted=true，encoder_zero 同公式 ---- */
    m.pole_pairs = 4; m.enc_inverted = true; m.mech_bias = 0.2f;
    m.wait_total_ms = 0u;
    CHECK(cal_abi_align_phase(&ctx) == 0);
    CHECK(ctx.inverted == true);
    CHECK_NEAR(ctx.encoder_zero, -0.2f * 4.0f, 1e-4f);

    /* ---- 场景3：非零对齐角 + 零偏 → encoder_zero 与 align_theta 无关（= -bias·pp） ---- */
    m.pole_pairs = 4; m.enc_inverted = false; m.mech_bias = 0.1f;
    ctx.align_theta_rad = 1.2f;
    CHECK(cal_abi_align_phase(&ctx) == 0);
    CHECK_NEAR(ctx.encoder_zero, -0.1f * 4.0f, 1e-4f);
    ctx.align_theta_rad = 0.0f;

    /* ---- 场景4：移动量不足（min_move 超大）→ 失败（转子未跟随） ---- */
    m.pole_pairs = 4; m.enc_inverted = false; m.mech_bias = 0.0f;
    ctx.min_move_rad = 10.0f;   /* 一圈机械角 ~1.57 < 10 → 判失败 */
    CHECK(cal_abi_align_phase(&ctx) != 0);
    ctx.min_move_rad = 0.0f;

    /* ---- 场景5：响应/极对数校验（对齐 SimpleFOC pp_check / ODrive scan_response） ---- */
    m.pole_pairs = 4; m.enc_inverted = false; m.mech_bias = 0.0f;
    ctx.pole_pairs = 8;    /* 电机实际 4 极对，配成 8 → moved·pp = 4π ≠ 2π → 拒绝 */
    CHECK(cal_abi_align_phase(&ctx) != 0);
    CHECK(ctx.pp_ok == false);
    ctx.pole_pairs = 4;    /* 恢复正确 → 校验通过 */
    CHECK(cal_abi_align_phase(&ctx) == 0);
    CHECK(ctx.pp_ok == true);

    /* ---- 非法入参 ---- */
    CHECK(cal_abi_align_phase(NULL) != 0);
    {
        CalAbiAlignCtx bad = {0};
        bad.set_elec_voltage = fake_set_voltage;
        bad.get_mech_angle = fake_get_mech;
        bad.wait_ms = fake_wait;
        bad.hw = &m;
        bad.pole_pairs = 4;
        bad.align_voltage = 0.0f;    /* 电压 0 非法 */
        CHECK(cal_abi_align_phase(&bad) != 0);
        bad.align_voltage = 1.0f;
        bad.pole_pairs = 0;          /* 极对数 0 非法 */
        CHECK(cal_abi_align_phase(&bad) != 0);
    }

    TEST_REPORT();
}
