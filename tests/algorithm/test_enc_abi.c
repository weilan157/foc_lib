/*
 * test_enc_abi.c —— ABZ 增量编码器单测（注入 fake 硬件）
 *
 * 覆盖：计数→角度、多圈累计（含 mod 环绕）、正反转、速度差分、
 *       index 找零（未找零前 STALE）、inverted 方向翻转。
 */
#include "device/encoder/enc_abi.h"
#include "../test_assert.h"

#define PI     (3.141592653589793f)
#define TWO_PI (6.283185307179586f)

/* ---- fake ABZ 硬件（可编程） ---- */
typedef struct {
    int32_t  count;
    bool     index;
    uint64_t tick_us;
} FakeAbzHw;

static int fake_abz_read_count(void *hw, int32_t *count)
{
    *count = ((FakeAbzHw *)hw)->count;
    return 0;
}

static bool fake_abz_read_index(void *hw)
{
    return ((FakeAbzHw *)hw)->index;
}

static uint64_t fake_abz_tick(void *hw)
{
    return ((FakeAbzHw *)hw)->tick_us;
}

int main(void)
{
    FakeAbzHw hw = {0, false, 0u};
    EncAbiHwOps hwops = {fake_abz_read_count, fake_abz_read_index, fake_abz_tick, &hw};
    EncAbiCtx ctx = {0};
    EncoderFeedback fb;

    ctx.hw_ops = &hwops;
    ctx.cpr = 4096;
    ctx.use_index = false;
    ctx.inverted = false;
    CHECK(enc_abi_init(&ctx) == 0);

    /* 初始：count=0 → 角度 0 */
    hw.count = 0; hw.tick_us = 0u;
    enc_abi_update(&ctx);
    enc_abi_get_feedback(&ctx, &fb);
    CHECK_NEAR(fb.mech_angle_rad, 0.0f, 1e-5f);
    CHECK(fb.revolution == 0);

    /* count=cpr/4 → 角度 π/2；0.01s → 速度 π/2/0.01 ≈ 157 rad/s */
    hw.count = 1024; hw.tick_us = 10000u;
    enc_abi_update(&ctx);
    enc_abi_get_feedback(&ctx, &fb);
    CHECK_NEAR(fb.mech_angle_rad, PI / 2.0f, 1e-4f);
    CHECK_NEAR(fb.velocity, (PI / 2.0f) / 0.01f, 0.5f);
    CHECK(fb.quality == ENC_QUALITY_GOOD);

    /* 反向：count 1024 → 512（-512）→ 角度 π/4 */
    hw.count = 512; hw.tick_us = 20000u;
    enc_abi_update(&ctx);
    enc_abi_get_feedback(&ctx, &fb);
    CHECK_NEAR(fb.mech_angle_rad, PI / 4.0f, 1e-4f);

    /* ---- 多圈 + mod 环绕：total 累计跨过 4096 → revolution=1 ---- */
    {
        FakeAbzHw h2 = {0, false, 0u};
        EncAbiHwOps o2 = {fake_abz_read_count, fake_abz_read_index, fake_abz_tick, &h2};
        EncAbiCtx c2 = {0};
        c2.hw_ops = &o2; c2.cpr = 4096; c2.use_index = false; c2.inverted = false;
        CHECK(enc_abi_init(&c2) == 0);

        h2.count = 0;    h2.tick_us = 1000u; enc_abi_update(&c2);   /* total=0      */
        h2.count = 1024; h2.tick_us = 2000u; enc_abi_update(&c2);   /* total=1024   */
        h2.count = 2048; h2.tick_us = 3000u; enc_abi_update(&c2);   /* total=2048   */
        h2.count = 3072; h2.tick_us = 4000u; enc_abi_update(&c2);   /* total=3072   */
        h2.count = 0;    h2.tick_us = 5000u; enc_abi_update(&c2);   /* 3072→0 跨零:+1024 → 4096 */
        CHECK(enc_abi_get_feedback(&c2, &fb) == 0);
        CHECK(fb.revolution == 1);
        CHECK_NEAR(fb.mech_angle_rad, TWO_PI, 1e-4f);

        h2.count = 904;  h2.tick_us = 6000u; enc_abi_update(&c2);   /* total=5000   */
        CHECK(enc_abi_get_feedback(&c2, &fb) == 0);
        CHECK(fb.revolution == 1);
        CHECK_NEAR(fb.mech_angle_rad, TWO_PI * 5000.0f / 4096.0f, 1e-4f);
    }

    /* ---- index 找零：找到前 STALE，找到后 GOOD 且归零累计 ---- */
    {
        FakeAbzHw h3 = {0, false, 0u};
        EncAbiHwOps o3 = {fake_abz_read_count, fake_abz_read_index, fake_abz_tick, &h3};
        EncAbiCtx c3 = {0};
        c3.hw_ops = &o3; c3.cpr = 1024; c3.use_index = true; c3.inverted = false;
        CHECK(enc_abi_init(&c3) == 0);

        h3.count = 256; h3.index = false; h3.tick_us = 100u;
        enc_abi_update(&c3);
        CHECK(c3.index_found == false);
        CHECK(c3.quality == ENC_QUALITY_STALE);

        h3.count = 512; h3.index = true; h3.tick_us = 200u;
        enc_abi_update(&c3);
        CHECK(c3.index_found == true);          /* index 触发 → 归零 */
        CHECK(c3.quality == ENC_QUALITY_GOOD);

        h3.index = false; h3.count = 612; h3.tick_us = 300u;   /* 找零后继续正向前进 100（512→612） */
        enc_abi_update(&c3);
        CHECK(enc_abi_get_feedback(&c3, &fb) == 0);
        CHECK_NEAR(fb.mech_angle_rad, TWO_PI * 100.0f / 1024.0f, 1e-4f);
    }

    /* ---- 每转 index 纠错：找零后若计数漂移，下一个 Z 把位置归一到整圈 ---- */
    {
        FakeAbzHw h5 = {0, false, 0u};
        EncAbiHwOps o5 = {fake_abz_read_count, fake_abz_read_index, fake_abz_tick, &h5};
        EncAbiCtx c5 = {0};
        c5.hw_ops = &o5; c5.cpr = 1024; c5.use_index = true; c5.inverted = false;
        CHECK(enc_abi_init(&c5) == 0);

        h5.index = true;  h5.count = 256; h5.tick_us = 100u;   /* 首次 Z → 找零 */
        enc_abi_update(&c5);
        CHECK(c5.index_found == true);
        CHECK(c5.total_count == 0);

        h5.index = false; h5.count = 356; h5.tick_us = 200u;   /* 正常累计 100 */
        enc_abi_update(&c5);
        CHECK(c5.total_count == 100);

        c5.total_count = 1016;                                 /* 注入漂移：应 1024，计数只到 1016 */

        h5.index = true; h5.count = 356; h5.tick_us = 300u;    /* 第2次 Z → 每转纠错 */
        enc_abi_update(&c5);
        CHECK(c5.total_count == 1024);                         /* 归一到整圈 */
        CHECK(enc_abi_get_feedback(&c5, &fb) == 0);
        CHECK_NEAR(fb.mech_angle_rad, TWO_PI, 1e-3f);
    }

    /* ---- 可选 PLL 位置/速度（ODrive 式临界阻尼）：匀速跟踪且平滑 ---- */
    {
        FakeAbzHw h6 = {0, false, 0u};
        EncAbiHwOps o6 = {fake_abz_read_count, fake_abz_read_index, fake_abz_tick, &h6};
        EncAbiCtx c6;
        int i;
        c6.hw_ops = &o6; c6.cpr = 4096; c6.use_index = false; c6.inverted = false;
        c6.use_pll = true; c6.pll_bandwidth_hz = 100.0f;
        CHECK(enc_abi_init(&c6) == 0);

        /* 匀速：每 1ms 前进 256 计数 → 实际速度 = (2π·256/4096)/0.001 ≈ 392.7 rad/s。
           40ms 充分收敛；PLL 对匀速加速有稳态滞后 ~ v/(2π·bw) ≈ 0.6 rad，容差放宽 */
        for (i = 0; i <= 40; i++) {
            h6.count = i * 256;
            h6.tick_us = (uint64_t)i * 1000u;
            enc_abi_update(&c6);
        }
        CHECK(enc_abi_get_feedback(&c6, &fb) == 0);
        CHECK_NEAR(fb.mech_angle_rad, TWO_PI * (float)(40 * 256) / 4096.0f, 1.0f);  /* ≈15.708 */
        CHECK(fb.velocity > 350.0f);
        CHECK(fb.velocity < 450.0f);
    }

    /* ---- inverted：读数反向 ---- */
    {
        FakeAbzHw h4 = {0, false, 0u};
        EncAbiHwOps o4 = {fake_abz_read_count, fake_abz_read_index, fake_abz_tick, &h4};
        EncAbiCtx c4 = {0};
        c4.hw_ops = &o4; c4.cpr = 1024; c4.use_index = false; c4.inverted = true;
        CHECK(enc_abi_init(&c4) == 0);

        h4.count = 0;   h4.tick_us = 100u; enc_abi_update(&c4);   /* 读 0 → -0 */
        h4.count = 100; h4.tick_us = 200u; enc_abi_update(&c4);   /* 读 100 → -100 */
        CHECK(enc_abi_get_feedback(&c4, &fb) == 0);
        CHECK_NEAR(fb.mech_angle_rad, -TWO_PI * 100.0f / 1024.0f, 1e-4f);
    }

    /* ---- 非法输入 ---- */
    CHECK(enc_abi_init(NULL) != 0);
    {
        EncAbiCtx bad = {0};
        bad.hw_ops = &hwops; bad.cpr = 0;   /* cpr=0 非法 */
        CHECK(enc_abi_init(&bad) != 0);
    }
    CHECK(enc_abi_update(NULL) != 0);
    CHECK(enc_abi_get_feedback(NULL, &fb) != 0);
    CHECK(enc_abi_get_feedback(&ctx, NULL) != 0);

    TEST_REPORT();
}
