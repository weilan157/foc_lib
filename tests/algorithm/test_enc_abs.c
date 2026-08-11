/*
 * test_enc_abs.c —— 总线无关绝对编码器单测（注入 fake 硬件）
 *
 * 覆盖：单圈 raw→角度、速度差分（含跨圈环绕修正）、inverted 反向、单次无效→STALE、
 *       连续无效→BAD、恢复→GOOD、错误率统计、多圈 read_multi（含跨圈速度）、
 *       可选 PLL、装配（ENCODER_TYPE_ABS）、非法输入。
 */
#include "device/encoder/enc_abs.h"
#include "device/encoder/encoder.h"
#include "../test_assert.h"

#define PI     (3.141592653589793f)
#define TWO_PI (6.283185307179586f)

/* ---- fake 绝对编码器硬件（可编程，模拟任意总线） ---- */
typedef struct {
    uint32_t raw;
    bool     data_valid;
    int32_t  turns;
    uint64_t tick_us;
} FakeAbsHw;

static int fake_read_abs(void *hw, uint32_t *raw, bool *valid)
{
    *raw = ((FakeAbsHw *)hw)->raw;
    *valid = ((FakeAbsHw *)hw)->data_valid;
    return 0;
}
static int fake_read_multi(void *hw, int32_t *turns)
{
    *turns = ((FakeAbsHw *)hw)->turns;
    return 0;
}
static uint64_t fake_tick(void *hw)
{
    return ((FakeAbsHw *)hw)->tick_us;
}

int main(void)
{
    FakeAbsHw hw = {0u, true, 0, 0u};
    EncAbsHwOps hwops = {fake_read_abs, NULL, fake_tick, &hw};
    EncAbsCtx ctx = {0};
    EncoderFeedback fb;

    ctx.hw_ops = &hwops;
    ctx.resolution = 16384;
    ctx.data_err_limit = 3;
    CHECK(enc_abs_init(&ctx) == 0);

    /* ① 单圈 raw=4096/16384 → 角度 π/2 */
    hw.raw = 4096; hw.data_valid = true; hw.tick_us = 0u;
    enc_abs_update(&ctx);
    enc_abs_get_feedback(&ctx, &fb);
    CHECK_NEAR(fb.mech_angle_rad, PI / 2.0f, 1e-4f);
    CHECK(fb.revolution == 0);
    CHECK(fb.quality == ENC_QUALITY_GOOD);

    /* ② 速度：0.01s 内 raw 4096→8192（Δ=π/2）→ ~157 rad/s */
    hw.raw = 8192; hw.tick_us = 10000u;
    enc_abs_update(&ctx);
    enc_abs_get_feedback(&ctx, &fb);
    CHECK_NEAR(fb.velocity, (PI / 2.0f) / 0.01f, 1.0f);
    CHECK_NEAR(fb.mech_angle_rad, PI, 1e-4f);

    /* ③ 跨圈环绕：raw 16383→1 仅移动 2 计数 → 不产生 -2π 速度尖峰 */
    hw.raw = 16383; hw.tick_us = 20000u;
    enc_abs_update(&ctx);
    hw.raw = 1; hw.tick_us = 30000u;
    enc_abs_update(&ctx);
    enc_abs_get_feedback(&ctx, &fb);
    CHECK_NEAR(fb.mech_angle_rad, TWO_PI * 1.0f / 16384.0f, 1e-4f);
    CHECK_NEAR(fb.velocity, (TWO_PI * 2.0f / 16384.0f) / 0.01f, 0.01f);

    /* ④ 单次无效 → STALE（沿用旧角度），连续 3 次 → BAD，恢复 → GOOD */
    hw.data_valid = false; hw.tick_us = 40000u;
    enc_abs_update(&ctx);
    enc_abs_get_feedback(&ctx, &fb);
    CHECK(fb.quality == ENC_QUALITY_STALE);
    CHECK_NEAR(fb.mech_angle_rad, TWO_PI * 1.0f / 16384.0f, 1e-4f);
    hw.tick_us = 50000u; enc_abs_update(&ctx);   /* 连续第 2 次 */
    hw.tick_us = 60000u; enc_abs_update(&ctx);   /* 连续第 3 次 → BAD */
    enc_abs_get_feedback(&ctx, &fb);
    CHECK(fb.quality == ENC_QUALITY_BAD);
    hw.raw = 8192; hw.data_valid = true; hw.tick_us = 70000u;   /* 恢复 */
    enc_abs_update(&ctx);
    enc_abs_get_feedback(&ctx, &fb);
    CHECK(fb.quality == ENC_QUALITY_GOOD);
    CHECK_NEAR(fb.mech_angle_rad, PI, 1e-4f);

    /* ⑤ inverted：单圈角度取反（raw 4096 → res-4096=12288 → 3π/2，等价 -π/2 mod 2π），
       速度反向（对照 SimpleFOC direction / enc_abi.inverted 相位校准语义） */
    {
        EncAbsCtx ci = {0};
        EncAbsHwOps ho = {fake_read_abs, NULL, fake_tick, &hw};
        ci.hw_ops = &ho; ci.resolution = 16384; ci.inverted = true;
        CHECK(enc_abs_init(&ci) == 0);
        hw.raw = 4096; hw.data_valid = true; hw.tick_us = 0u;
        enc_abs_update(&ci);
        enc_abs_get_feedback(&ci, &fb);
        CHECK_NEAR(fb.mech_angle_rad, 3.0f * PI / 2.0f, 1e-4f);   /* 取反 → 3π/2 */
        hw.raw = 8192; hw.tick_us = 10000u;
        enc_abs_update(&ci);
        enc_abs_get_feedback(&ci, &fb);
        CHECK_NEAR(fb.velocity, -(PI / 2.0f) / 0.01f, 1.0f);    /* 反向 */
    }

    /* ⑥ 错误率统计（对照 VESC encoder_get_error_rate） */
    {
        EncAbsCtx ce = {0};
        EncAbsHwOps ho = {fake_read_abs, NULL, fake_tick, &hw};
        ce.hw_ops = &ho; ce.resolution = 1024;
        CHECK(enc_abs_init(&ce) == 0);
        hw.raw = 512;  hw.data_valid = true;  hw.tick_us = 0u;
        enc_abs_update(&ce);                                   /* 0/1 */
        hw.data_valid = false; hw.tick_us = 10000u;
        enc_abs_update(&ce);                                   /* 1/2 */
        hw.tick_us = 20000u;
        enc_abs_update(&ce);                                   /* 2/3 */
        hw.raw = 256; hw.data_valid = true; hw.tick_us = 30000u;
        enc_abs_update(&ce);                                   /* 2/4 */
        CHECK_NEAR(ce.err_rate, 0.5f, 1e-4f);
        CHECK(ce.err_count_total == 2u);
    }

    /* ⑦ 多圈 read_multi（独立圈数寄存器，如 EnDat/BiSS 多圈） */
    {
        EncAbsHwOps ho = {fake_read_abs, fake_read_multi, fake_tick, &hw};
        EncAbsCtx cm = {0};
        cm.hw_ops = &ho; cm.resolution = 16384;
        CHECK(enc_abs_init(&cm) == 0);
        hw.raw = 4096; hw.turns = 2; hw.data_valid = true; hw.tick_us = 0u;
        enc_abs_update(&cm);
        enc_abs_get_feedback(&cm, &fb);
        CHECK(fb.revolution == 2);
        CHECK_NEAR(fb.mech_angle_rad, 2.0f * TWO_PI + PI / 2.0f, 1e-3f);
        /* 跨圈：raw 16383/圈2 → raw 0/圈3 → 一整圈，速度 ≈ 2π/0.01 */
        hw.raw = 16383; hw.turns = 2; hw.tick_us = 10000u;
        enc_abs_update(&cm);
        hw.raw = 0; hw.turns = 3; hw.tick_us = 20000u;
        enc_abs_update(&cm);
        enc_abs_get_feedback(&cm, &fb);
        CHECK_NEAR(fb.velocity, TWO_PI / 0.01f, 5.0f);
        CHECK(fb.revolution == 3);
    }

    /* ⑧ 可选 PLL：匀速跟踪收敛（ODrive 式临界阻尼） */
    {
        EncAbsCtx cp = {0};
        uint32_t i;
        cp.hw_ops = &hwops; cp.resolution = 16384;
        cp.use_pll = true; cp.pll_bandwidth_hz = 100.0f;
        CHECK(enc_abs_init(&cp) == 0);
        /* 匀速：每 1ms raw +32 计数（= 12.27 rad/s） */
        hw.data_valid = true;
        for (i = 0u; i < 60u; i++) {
            hw.raw = 32u * (i + 1u);
            hw.tick_us = 1000u * (i + 1u);
            enc_abs_update(&cp);
        }
        enc_abs_get_feedback(&cp, &fb);
        CHECK_NEAR(fb.velocity, (TWO_PI * 32.0f / 16384.0f) / 0.001f, 2.0f);
        CHECK_NEAR(fb.mech_angle_rad, TWO_PI * (32.0f * 60.0f) / 16384.0f, 0.1f);
        CHECK(fb.quality == ENC_QUALITY_GOOD);
    }

    /* ⑨ 装配：encoder_ops_for_type */
    {
        const PositionSensorOps *ops;
        ops = encoder_ops_for_type(ENCODER_TYPE_ABZ);
        CHECK(ops == &enc_abi_ops);
        ops = encoder_ops_for_type(ENCODER_TYPE_ABS);
        CHECK(ops == &enc_abs_ops);
        ops = encoder_ops_for_type((EncoderType)ENCODER_TYPE_COUNT);
        CHECK(ops == NULL);
    }

    /* ⑩ 非法输入 */
    CHECK(enc_abs_init(NULL) != 0);
    {
        EncAbsCtx bad = {0};
        bad.hw_ops = &hwops; bad.resolution = 0;   /* resolution=0 非法 */
        CHECK(enc_abs_init(&bad) != 0);
    }
    CHECK(enc_abs_update(NULL) != 0);
    CHECK(enc_abs_get_feedback(NULL, &fb) != 0);
    CHECK(enc_abs_get_feedback(&ctx, NULL) != 0);

    TEST_REPORT();
}
