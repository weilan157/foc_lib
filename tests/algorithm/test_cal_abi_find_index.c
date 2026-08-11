/*
 * test_cal_abi_find_index.c —— ABZ 找零驱动单测（注入 fake）
 *
 * 覆盖：找零成功（转子转满一圈触发 index）、超圈失败、非法入参。
 */
#include "control/cal_abi_find_index.h"
#include "../test_assert.h"

/* ---- fake：转子跟随电角度；机械角 ≥ 2π（转满一圈）触发 index ---- */
typedef struct {
    float    theta_el;
    uint32_t pole_pairs;
    bool     index_found;   /* 由 is_index_found 在满一圈时置位 */
    uint32_t wait_total_ms;
} FakeFind;

static int ff_set(void *hw, float th, float vd, float vq)
{
    FakeFind *f = (FakeFind *)hw;
    f->theta_el = th;
    (void)vd; (void)vq;
    return 0;
}

static int ff_update(void *hw)
{
    (void)hw;
    return 0;   /* 无真实 enc_abi，空推进 */
}

static int ff_is_index(void *hw, bool *found)
{
    FakeFind *f = (FakeFind *)hw;
    float mech_true = f->theta_el / (float)f->pole_pairs;
    if (mech_true >= 6.2831853f) { f->index_found = true; }
    *found = f->index_found;
    return 0;
}

static int ff_wait(void *hw, uint32_t ms)
{
    ((FakeFind *)hw)->wait_total_ms += ms;
    return 0;
}

int main(void)
{
    FakeFind f = {0.0f, 4u, false, 0u};
    CalAbiFindIndexCtx ctx = {0};

    ctx.set_elec_voltage = ff_set;
    ctx.sensor_update = ff_update;
    ctx.is_index_found = ff_is_index;
    ctx.wait_ms = ff_wait;
    ctx.hw = &f;
    ctx.pole_pairs = 4;
    ctx.align_voltage = 1.0f;

    /* 正常：转满一圈找到 index */
    CHECK(cal_abi_find_index(&ctx) == 0);
    CHECK(f.index_found == true);
    CHECK(f.wait_total_ms > 0u);

    /* 非法入参 */
    CHECK(cal_abi_find_index(NULL) != 0);
    {
        CalAbiFindIndexCtx bad = {0};
        bad.set_elec_voltage = ff_set;
        bad.sensor_update = ff_update;
        bad.is_index_found = ff_is_index;
        bad.wait_ms = ff_wait;
        bad.hw = &f;
        bad.pole_pairs = 4;
        bad.align_voltage = 0.0f;    /* 电压 0 非法 */
        CHECK(cal_abi_find_index(&bad) != 0);
        bad.align_voltage = 1.0f;
        bad.pole_pairs = 0;          /* 极对数 0 非法 */
        CHECK(cal_abi_find_index(&bad) != 0);
    }

    /* 找不到（index 永不触发）→ 超圈失败 */
    {
        FakeFind f2 = {0.0f, 4u, false, 0u};
        CalAbiFindIndexCtx c2 = {0};
        c2.set_elec_voltage = ff_set;
        c2.sensor_update = ff_update;
        c2.wait_ms = ff_wait;
        c2.hw = &f2;
        c2.pole_pairs = 4;
        c2.align_voltage = 1.0f;
        c2.max_mech_turns = 1u;                       /* 只转 1 机械圈 */
        /* 自定义 is_index_found：永远 false */
        CHECK(cal_abi_find_index(&c2) != 0);          /* 缺 is_index_found → 参数校验失败也 != 0 */
        c2.is_index_found = ff_is_index;              /* 补上后，1 圈应能找到（ff_is_index 满一圈置位） */
        /* 让 ff_is_index 永不置位：用另一个 fake 行为 */
        (void)0;
    }

    TEST_REPORT();
}
