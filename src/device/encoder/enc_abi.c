/*
 * enc_abi.c —— ABZ 增量编码器实现
 */
#include "device/encoder/enc_abi.h"
#include "foc_types.h"
#include <math.h>   /* llroundf：每转 index 归一到整圈 */

#define ENC_ABI_TWO_PI  (6.283185307179586f)
#define ENC_ABI_DT_MAX  (1000000u)   /* 1s：异常 dt 过滤 */

/* 环绕安全的增量：|delta| > cpr/2 视为跨零环绕，修正到 ±cpr/2 内。
   half/cpr 由调用方预计算传入（避免每周期整数除法） */
static int32_t enc_abi_delta(int32_t cur, int32_t prev, int32_t half, int32_t cpr)
{
    int32_t d;

    d = cur - prev;
    if (d > half) {
        d -= cpr;
    } else if (d < -half) {
        d += cpr;
    }
    return d;
}

int enc_abi_init(void *ctx)
{
    EncAbiCtx *c = (EncAbiCtx *)ctx;

    if ((c == NULL) || (c->hw_ops == NULL) || (c->cpr == 0u)) {
        return FOC_ERROR;
    }
    if ((c->hw_ops->read_count == NULL) || (c->hw_ops->get_tick_us == NULL)) {
        return FOC_ERROR;
    }

    c->total_count    = 0;
    c->last_count     = 0;
    c->last_angle_rad = 0.0f;
    c->velocity       = 0.0f;
    c->last_tick_us   = 0u;
    c->index_found    = !c->use_index;   /* 不用 index 则直接可用 */
    c->first          = true;
    c->quality        = ENC_QUALITY_GOOD;

    /* 预计算（Fast Loop 性能：cpr 不变，避免每周期整数/浮点除法） */
    c->half_cpr = (int32_t)(c->cpr / 2u);
    c->scale    = ENC_ABI_TWO_PI / (float)c->cpr;

    /* PLL 初始化（ODrive 式：kp=2·bw，ki=0.25·kp²，临界阻尼） */
    c->pll_pos   = 0.0f;
    c->pll_vel   = 0.0f;
    c->pll_kp    = 0.0f;
    c->pll_ki    = 0.0f;
    c->pll_first = true;
    if (c->use_pll && (c->pll_bandwidth_hz > 0.0f)) {
        c->pll_kp = 2.0f * c->pll_bandwidth_hz;
        c->pll_ki = 0.25f * c->pll_kp * c->pll_kp;
    }
    return FOC_OK;
}

int enc_abi_update(void *ctx)
{
    EncAbiCtx *c = (EncAbiCtx *)ctx;
    int32_t count = 0;
    uint64_t now_us;
    float angle;

    if ((c == NULL) || (c->hw_ops == NULL)) {
        return FOC_ERROR;
    }

    now_us = c->hw_ops->get_tick_us(c->hw_ops->hw);
    if (c->hw_ops->read_count(c->hw_ops->hw, &count) != FOC_OK) {
        c->quality = ENC_QUALITY_BAD;   /* 计数读取失败 → 交 Fault */
        return FOC_OK;
    }
    if (c->inverted) {
        count = -count;
    }

    if (!c->first) {
        c->total_count += (int64_t)enc_abi_delta(count, c->last_count, c->half_cpr, (int32_t)c->cpr);
    }
    c->last_count = count;
    c->first = false;

    /* index：首次找零 + 每转纠错（对齐 VESC/ODrive/SimpleFOC，消除累计漂移） */
    if (c->use_index && (c->hw_ops->read_index != NULL) && c->hw_ops->read_index(c->hw_ops->hw)) {
        if (!c->index_found) {
            c->total_count = 0;                 /* 首次：建立绝对零位 */
            c->index_found = true;
        } else {
            /* 每转：归一到最近整圈；平移 last_angle 防速度尖峰 */
            int64_t aligned;
            aligned = (int64_t)llroundf((float)c->total_count / (float)c->cpr) * (int64_t)c->cpr;
            c->last_angle_rad += (float)(aligned - c->total_count) * c->scale;
            c->total_count = aligned;
        }
    }

    angle = (float)c->total_count * c->scale;

    /* 位置/速度：可选 PLL（ODrive 式临界阻尼）vs 一阶差分 */
    if (c->use_pll) {
        if (c->pll_first) {
            c->pll_pos = angle;                 /* 首次：以当前角度为 PLL 初值 */
            c->pll_vel = 0.0f;
            c->pll_first = false;
        } else if (now_us > c->last_tick_us) {
            uint64_t du = now_us - c->last_tick_us;
            if (du < ENC_ABI_DT_MAX) {
                float dts = (float)du * 1e-6f;
                float delta = angle - c->pll_pos;
                c->pll_vel += dts * c->pll_ki * delta;
                c->pll_pos += dts * (c->pll_vel + (c->pll_kp * delta));
            }
        }
    } else if (now_us > c->last_tick_us) {
        uint64_t du = now_us - c->last_tick_us;
        if (du < ENC_ABI_DT_MAX) {
            float dts = (float)du * 1e-6f;
            c->velocity = (angle - c->last_angle_rad) / dts;
        }
    }
    c->last_angle_rad = angle;
    c->last_tick_us = now_us;

    c->quality = ((!c->use_index) || c->index_found) ? ENC_QUALITY_GOOD : ENC_QUALITY_STALE;
    return FOC_OK;
}

int enc_abi_get_feedback(void *ctx, EncoderFeedback *fb)
{
    EncAbiCtx *c = (EncAbiCtx *)ctx;

    if ((c == NULL) || (fb == NULL)) {
        return FOC_ERROR;
    }

    if (c->use_pll && !c->pll_first) {
        fb->mech_angle_rad = c->pll_pos;        /* PLL 平滑位置 */
        fb->velocity       = c->pll_vel;        /* PLL 速度 */
    } else {
        fb->mech_angle_rad = ENC_ABI_TWO_PI * (float)c->total_count / (float)c->cpr;
        fb->velocity       = c->velocity;
    }
    fb->revolution = (int32_t)(c->total_count / (int64_t)c->cpr);
    fb->quality    = c->quality;
    return FOC_OK;
}

const PositionSensorOps enc_abi_ops = {
    .init         = enc_abi_init,
    .update       = enc_abi_update,
    .get_feedback = enc_abi_get_feedback,
};
