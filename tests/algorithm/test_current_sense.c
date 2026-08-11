/*
 * test_current_sense.c —— 相电流感测单测（注入 fake 相电流硬件）
 *
 * 覆盖：raw→电流（零点/增益校准）、单次过流降级、连续过流→BAD、恢复→GOOD、错误率。
 */
#include "device/current_sense.h"
#include "../test_assert.h"

/* ---- fake 相电流硬件 ---- */
typedef struct {
    float raw_a, raw_b, raw_c;
    bool  read_fail;
} FakeCsHw;

static int fake_read_raw(void *hw, float *a, float *b, float *c)
{
    FakeCsHw *f = (FakeCsHw *)hw;
    if (f->read_fail) { return 1; }
    *a = f->raw_a; *b = f->raw_b; *c = f->raw_c;
    return 0;
}
static uint64_t fake_tick(void *hw) { (void)hw; return 1000u; }

int main(void)
{
    FakeCsHw hw = {0.5f, 0.5f, 0.5f, false};
    CurrentSenseHwOps hwops = {fake_read_raw, fake_tick, &hw};
    CurrentSenseCtx ctx = {0};
    SampleFrame sf;

    ctx.hw_ops = &hwops;
    ctx.gain_a = ctx.gain_b = ctx.gain_c = 1.0f;      /* 1A/V */
    ctx.offset_a = ctx.offset_b = ctx.offset_c = 0.5f; /* 零点 0.5V */
    ctx.max_current = 5.0f;
    ctx.overcurrent_limit = 2u;   /* 连续 2 次过流 → BAD */
    CHECK(current_sense_init(&ctx) == 0);

    /* ① 零电流：raw=0.5 → i=0 */
    hw.raw_a = hw.raw_b = hw.raw_c = 0.5f;
    CHECK(current_sense_reconstruct(&ctx, &sf) == 0);
    CHECK_NEAR(sf.ia, 0.0f, 1e-4f);
    CHECK_NEAR(sf.ib, 0.0f, 1e-4f);
    CHECK_NEAR(sf.ic, 0.0f, 1e-4f);
    CHECK(ctx.quality == ENC_QUALITY_GOOD);

    /* ② raw=1.5 → i=1.0A */
    hw.raw_a = hw.raw_b = hw.raw_c = 1.5f;
    CHECK(current_sense_reconstruct(&ctx, &sf) == 0);
    CHECK_NEAR(sf.ia, 1.0f, 1e-4f);
    CHECK_NEAR(sf.ib, 1.0f, 1e-4f);
    CHECK_NEAR(sf.ic, 1.0f, 1e-4f);

    /* ③ 单次过流（i=6A > 5A）：降级（沿用旧值），不立即 BAD */
    hw.raw_a = hw.raw_b = hw.raw_c = 6.5f;
    CHECK(current_sense_reconstruct(&ctx, &sf) == 0);
    CHECK(ctx.quality == ENC_QUALITY_GOOD);           /* 1 次 < limit=3 */
    /* 连续 3 次 → BAD → 返回错误 */
    hw.raw_a = hw.raw_b = hw.raw_c = 6.5f;
    CHECK(current_sense_reconstruct(&ctx, &sf) != 0);
    CHECK(ctx.quality == ENC_QUALITY_BAD);
    CHECK_NEAR(ctx.last_ia, 1.0f, 1e-4f);             /* 沿用旧 GOOD 值（禁置 0） */

    /* ④ 恢复 → GOOD */
    hw.raw_a = hw.raw_b = hw.raw_c = 1.0f;
    CHECK(current_sense_reconstruct(&ctx, &sf) == 0);
    CHECK(ctx.quality == ENC_QUALITY_GOOD);
    CHECK_NEAR(sf.ia, 0.5f, 1e-4f);

    /* ⑤ 读取失败 → BAD */
    hw.read_fail = true;
    CHECK(current_sense_reconstruct(&ctx, &sf) != 0);
    CHECK(ctx.quality == ENC_QUALITY_BAD);
    hw.read_fail = false;

    /* ⑥ 错误率 */
    CHECK(ctx.err_count_total > 0u);
    CHECK(ctx.err_rate > 0.0f);

    /* ⑦ 非法输入 */
    CHECK(current_sense_init(NULL) != 0);
    {
        CurrentSenseCtx bad = {0};
        bad.hw_ops = &hwops;
        bad.gain_a = 0.0f; bad.gain_b = 1.0f; bad.gain_c = 1.0f;   /* gain 0 非法 */
        CHECK(current_sense_init(&bad) != 0);
    }
    CHECK(current_sense_reconstruct(NULL, &sf) != 0);
    CHECK(current_sense_reconstruct(&ctx, NULL) != 0);

    TEST_REPORT();
}
