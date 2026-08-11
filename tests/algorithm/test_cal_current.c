/*
 * test_cal_current.c —— 电流零点校准单测（注入 fake 相电流硬件）
 *
 * 覆盖：零电压采样均值 = 偏移、rounds 配置、稳定等待、非法入参。
 */
#include "control/cal_current.h"
#include "../test_assert.h"

/* ---- fake：三相原始值固定，带轻微噪声 ---- */
typedef struct {
    float a, b, c;
    uint32_t wait_ms_total;
    uint32_t zero_calls;
    bool fail_read;
} FakeHw;

static int fake_zero(void *hw) { ((FakeHw *)hw)->zero_calls++; return 0; }
static int fake_read(void *hw, float *a, float *b, float *c)
{
    FakeHw *f = (FakeHw *)hw;
    if (f->fail_read) { return 1; }
    *a = f->a; *b = f->b; *c = f->c;
    return 0;
}
static int fake_wait(void *hw, uint32_t ms)
{
    ((FakeHw *)hw)->wait_ms_total += ms;
    return 0;
}

int main(void)
{
    FakeHw hw = {0.50f, 0.60f, 0.40f, 0u, 0u, false};
    CalCurrentCtx ctx = {0};

    ctx.set_zero_voltage = fake_zero;
    ctx.read_phase_raw = fake_read;
    ctx.wait_ms = fake_wait;
    ctx.hw = &hw;
    ctx.rounds = 32u;

    CHECK(cal_current_offset(&ctx) == 0);
    CHECK_NEAR(ctx.offset_a, 0.50f, 1e-4f);
    CHECK_NEAR(ctx.offset_b, 0.60f, 1e-4f);
    CHECK_NEAR(ctx.offset_c, 0.40f, 1e-4f);
    CHECK(hw.zero_calls == 1u);               /* 只施加一次零电压 */
    CHECK(hw.wait_ms_total >= 50u);           /* settle_ms 默认 50 */

    /* 配置 rounds/settle 默认 */
    {
        CalCurrentCtx c2 = {0};
        c2.set_zero_voltage = fake_zero;
        c2.read_phase_raw = fake_read;
        c2.wait_ms = fake_wait;
        c2.hw = &hw;
        hw.wait_ms_total = 0u;
        CHECK(cal_current_offset(&c2) == 0);  /* rounds=0 → 64，settle=0 → 50 */
        CHECK(hw.wait_ms_total == 50u);
        CHECK_NEAR(c2.offset_a, 0.50f, 1e-4f);
    }

    /* 读取失败 → 失败 */
    hw.fail_read = true;
    CHECK(cal_current_offset(&ctx) != 0);
    hw.fail_read = false;

    /* 非法入参 */
    CHECK(cal_current_offset(NULL) != 0);
    {
        CalCurrentCtx bad = {0};
        bad.read_phase_raw = fake_read;       /* 缺 set_zero_voltage/wait_ms */
        CHECK(cal_current_offset(&bad) != 0);
    }

    TEST_REPORT();
}
