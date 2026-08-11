/*
 * enc_abs.c —— 绝对编码器实现（总线无关）
 *
 * 语义层与总线解耦：HwOps.read_abs/read_multi/get_tick_us 由 board/HAL 注入，
 * 本文件只做"绝对编码器"的通用处理 —— raw→角度、数据有效→质量、方向、多圈、
 * 速度（跨圈环绕差分 / 可选 PLL）、错误率统计。
 */
#include "device/encoder/enc_abs.h"
#include "foc_types.h"
#include <math.h>   /* llroundf（仅 PLL 不需；保留以对齐 enc_abi） */

#define ENC_ABS_TWO_PI (6.283185307179586f)
#define ENC_ABS_DT_MAX (1000000u)   /* 1s：异常 dt 过滤 */

int enc_abs_init(void *ctx)
{
    EncAbsCtx *c = (EncAbsCtx *)ctx;

    if ((c == NULL) || (c->hw_ops == NULL) || (c->hw_ops->read_abs == NULL)) {
        return FOC_ERROR;
    }
    if (c->hw_ops->get_tick_us == NULL) {
        return FOC_ERROR;
    }
    if (c->resolution == 0u) {
        return FOC_ERROR;
    }
    if (c->data_err_limit == 0u) {
        c->data_err_limit = 1u;
    }

    c->last_raw        = 0u;
    c->data_err_count  = 0u;
    c->update_count    = 0u;
    c->err_count_total = 0u;
    c->err_rate        = 0.0f;
    c->last_turn       = 0;
    c->last_angle_rad  = 0.0f;
    c->velocity        = 0.0f;
    c->last_tick_us    = 0u;
    c->first           = true;
    c->quality         = ENC_QUALITY_GOOD;

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

int enc_abs_update(void *ctx)
{
    EncAbsCtx *c = (EncAbsCtx *)ctx;
    uint32_t raw = 0u;
    bool data_valid = false;
    uint64_t now_us;
    uint32_t raw_ang;
    int32_t turns;
    float angle_total;

    if ((c == NULL) || (c->hw_ops == NULL)) {
        return FOC_ERROR;
    }

    now_us = c->hw_ops->get_tick_us(c->hw_ops->hw);
    c->update_count++;                                  /* 总读取计数（含有效/无效） */

    if (c->hw_ops->read_abs(c->hw_ops->hw, &raw, &data_valid) != FOC_OK) {
        c->data_err_count++;
        c->err_count_total++;
        c->quality = ENC_QUALITY_BAD;                   /* 读取失败（超时等）→ 交 Fault */
        c->err_rate = (float)c->err_count_total / (float)c->update_count;
        return FOC_OK;
    }

    if (!data_valid) {
        c->data_err_count++;
        c->err_count_total++;
        c->quality = (c->data_err_count >= c->data_err_limit)
                     ? ENC_QUALITY_BAD                  /* 连续错 → BAD（交 Fault，禁 angle=0） */
                     : ENC_QUALITY_STALE;               /* 单次错 → 降级（沿用旧角度） */
        c->err_rate = (float)c->err_count_total / (float)c->update_count;
        return FOC_OK;                                  /* 不更新 last_raw / 角度 */
    }

    c->data_err_count = 0u;
    c->quality = ENC_QUALITY_GOOD;
    c->err_rate = (float)c->err_count_total / (float)c->update_count;

    /* 单圈原始值 + 圈数（read_multi 独立圈数；否则 raw 含圈数） */
    raw_ang = raw % c->resolution;
    turns = (int32_t)(raw / c->resolution);
    if (c->hw_ops->read_multi != NULL) {
        if (c->hw_ops->read_multi(c->hw_ops->hw, &turns) != FOC_OK) {
            c->data_err_count++;
            c->err_count_total++;
            c->quality = ENC_QUALITY_BAD;
            c->err_rate = (float)c->err_count_total / (float)c->update_count;
            return FOC_OK;
        }
    }

    /* 方向：inverted → 单圈角度取反（raw 取反到 [0,res)，标准绝对编码器做法；
       对照 SimpleFOC direction / enc_abi.inverted；取反后角度 ∈ [0,2π)，符号正确） */
    if (c->inverted) {
        raw_ang = (raw_ang == 0u) ? 0u : (c->resolution - raw_ang);
    }

    /* 多圈连续机械角（可超 2π） */
    angle_total = (float)turns * ENC_ABS_TWO_PI
                + ENC_ABS_TWO_PI * (float)raw_ang / (float)c->resolution;

    /* 首次：只建基准（不产出位移/速度） */
    if (c->first) {
        c->last_raw      = raw_ang;
        c->last_turn     = turns;
        c->last_angle_rad = angle_total;
        c->last_tick_us   = now_us;
        c->first          = false;
        if (c->use_pll) {
            c->pll_pos   = angle_total;
            c->pll_vel   = 0.0f;
            c->pll_first = false;
        }
        return FOC_OK;
    }

    /* 速度：单圈带符号增量（跨圈环绕修正）+ 圈数增量（read_multi）。
       raw_ang 已含 inverted 取反，故差分符号自然正确。 */
    {
        int32_t d_raw = (int32_t)raw_ang - (int32_t)c->last_raw;
        int32_t half  = (int32_t)(c->resolution / 2u);
        int32_t d_turns;
        float d_angle;

        if (d_raw > half) {
            d_raw -= (int32_t)c->resolution;
        } else if (d_raw < -half) {
            d_raw += (int32_t)c->resolution;
        }

        d_turns = turns - c->last_turn;

        d_angle = ENC_ABS_TWO_PI * ((float)d_raw / (float)c->resolution + (float)d_turns);

        if (c->use_pll) {
            /* PLL 平滑（ODrive 式临界阻尼，位置源为多圈连续角度） */
            if (now_us > c->last_tick_us) {
                uint64_t du = now_us - c->last_tick_us;
                if (du < ENC_ABS_DT_MAX) {
                    float dts = (float)du * 1e-6f;
                    float delta = angle_total - c->pll_pos;
                    c->pll_vel += dts * c->pll_ki * delta;
                    c->pll_pos += dts * (c->pll_vel + (c->pll_kp * delta));
                }
            }
        } else if (now_us > c->last_tick_us) {
            uint64_t du = now_us - c->last_tick_us;
            if (du < ENC_ABS_DT_MAX) {
                float dts = (float)du * 1e-6f;
                c->velocity = d_angle / dts;
            }
        }
    }

    c->last_raw      = raw_ang;   /* 速度差分之后才更新（保持"上一次"语义） */
    c->last_turn     = turns;
    c->last_angle_rad = angle_total;
    c->last_tick_us   = now_us;
    return FOC_OK;
}

int enc_abs_get_feedback(void *ctx, EncoderFeedback *fb)
{
    EncAbsCtx *c = (EncAbsCtx *)ctx;

    if ((c == NULL) || (fb == NULL)) {
        return FOC_ERROR;
    }

    if (c->use_pll && !c->pll_first) {
        fb->mech_angle_rad = c->pll_pos;        /* PLL 平滑位置（多圈连续） */
        fb->velocity       = c->pll_vel;        /* PLL 速度 */
    } else {
        fb->mech_angle_rad = c->last_angle_rad;
        fb->velocity       = c->velocity;
    }
    fb->revolution = (int32_t)(fb->mech_angle_rad / ENC_ABS_TWO_PI);   /* 圈数（向零截断） */
    fb->quality    = c->quality;
    return FOC_OK;
}

const PositionSensorOps enc_abs_ops = {
    .init         = enc_abs_init,
    .update       = enc_abs_update,
    .get_feedback = enc_abs_get_feedback,
};
