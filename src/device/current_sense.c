/*
 * current_sense.c —— 相电流感测实现（总线无关）
 *
 * raw(电压) → (raw − offset)·gain → 相电流 [A] → SampleFrame。
 * 过流检测：|i| > max_current 连续 overcurrent_limit 次 → BAD（交 Fault，禁置 0）。
 */
#include "device/current_sense.h"
#include "foc_types.h"

int current_sense_init(void *ctx)
{
    CurrentSenseCtx *c = (CurrentSenseCtx *)ctx;

    if ((c == NULL) || (c->hw_ops == NULL) || (c->hw_ops->read_phase_raw == NULL)) {
        return FOC_ERROR;
    }
    if ((c->gain_a <= 0.0f) || (c->gain_b <= 0.0f) || (c->gain_c <= 0.0f)) {
        return FOC_ERROR;
    }
    if (c->overcurrent_limit == 0u) {
        c->overcurrent_limit = 1u;
    }

    c->last_ia = 0.0f;
    c->last_ib = 0.0f;
    c->last_ic = 0.0f;
    c->overcurrent_count = 0u;
    c->read_count = 0u;
    c->err_count_total = 0u;
    c->err_rate = 0.0f;
    c->quality = ENC_QUALITY_GOOD;
    return FOC_OK;
}

static bool cs_overcurrent(CurrentSenseCtx *c, float ia, float ib, float ic)
{
    if (c->max_current <= 0.0f) { return false; }   /* 未启用检测 */
    return ((ia >  c->max_current) || (ia < -c->max_current) ||
            (ib >  c->max_current) || (ib < -c->max_current) ||
            (ic >  c->max_current) || (ic < -c->max_current));
}

int current_sense_reconstruct(void *ctx, SampleFrame *sf)
{
    CurrentSenseCtx *c = (CurrentSenseCtx *)ctx;
    float raw_a = 0.0f, raw_b = 0.0f, raw_c = 0.0f;
    float ia, ib, ic;
    uint64_t now_us;

    if ((c == NULL) || (sf == NULL) || (c->hw_ops == NULL)) {
        return FOC_ERROR;
    }

    now_us = c->hw_ops->get_tick_us(c->hw_ops->hw);
    c->read_count++;

    if (c->hw_ops->read_phase_raw(c->hw_ops->hw, &raw_a, &raw_b, &raw_c) != FOC_OK) {
        c->err_count_total++;
        c->quality = ENC_QUALITY_BAD;               /* 读取失败 → 交 Fault */
        c->err_rate = (float)c->err_count_total / (float)c->read_count;
        return FOC_ERROR;
    }

    /* 零点/增益校准 → 相电流 */
    ia = (raw_a - c->offset_a) * c->gain_a;
    ib = (raw_b - c->offset_b) * c->gain_b;
    ic = (raw_c - c->offset_c) * c->gain_c;

    if (cs_overcurrent(c, ia, ib, ic)) {
        c->overcurrent_count++;
        if (c->overcurrent_count >= c->overcurrent_limit) {
            c->err_count_total++;
            c->quality = ENC_QUALITY_BAD;           /* 连续过流 → BAD（交 Fault，禁置 0） */
            c->err_rate = (float)c->err_count_total / (float)c->read_count;
            return FOC_ERROR;
        }
        c->err_rate = (float)c->err_count_total / (float)c->read_count;
        return FOC_OK;                              /* 单次过流：沿用旧值（降级） */
    }

    /* 正常 */
    c->overcurrent_count = 0u;
    c->quality = ENC_QUALITY_GOOD;
    c->err_rate = (float)c->err_count_total / (float)c->read_count;
    c->last_ia = ia;
    c->last_ib = ib;
    c->last_ic = ic;

    sf->ia = ia;
    sf->ib = ib;
    sf->ic = ic;
    sf->timestamp_us = now_us;
    sf->cycle = c->read_count;
    return FOC_OK;
}
