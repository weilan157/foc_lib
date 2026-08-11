/*
 * abs_board.c —— 仿真绝对编码器板实现（理想电机 + 仿真绝对编码器）
 *
 * 模型假设（仅测试）：
 *   - 转子瞬时跟随施加电角度：mech_true = theta_el / pole_pairs（无惯性/无负载）
 *   - 编码器连续机械读数 = ±(mech_true + mech_bias)，方向由 enc_inverted 决定（board 层连续化）
 *   - 绝对原始值 raw = 连续读数归一化到 [0, 2π) 再映射到 [0, resolution)，无 index
 */
#include "simulation/abs_board.h"
#include <math.h>   /* fmodf */

#define ABS_BOARD_TWO_PI (6.283185307179586f)

void abs_board_init(AbsBoard *b, uint32_t pole_pairs, float mech_bias,
                    bool enc_inverted, uint32_t resolution)
{
    if (b == NULL) { return; }
    b->pole_pairs    = (pole_pairs == 0u) ? 1u : pole_pairs;
    b->mech_bias     = mech_bias;
    b->enc_inverted  = enc_inverted;
    b->resolution    = (resolution == 0u) ? 16384u : resolution;
    b->theta_el      = 0.0f;
    b->mech_true     = 0.0f;
    b->mech_raw      = 0.0f;
    b->raw           = 0u;
    b->tick_us       = 0u;
    b->set_voltage_calls = 0u;
}

int abs_board_set_elec_voltage(void *b, float theta_el_rad, float vd, float vq)
{
    AbsBoard *ab = (AbsBoard *)b;
    float m;
    if (ab == NULL) { return 1; }
    (void)vd;
    (void)vq;

    ab->theta_el = theta_el_rad;
    ab->mech_true = theta_el_rad / (float)ab->pole_pairs;

    /* 编码器连续机械读数（含零偏；方向按 enc_inverted 模拟硬件接线） */
    ab->mech_raw = ab->enc_inverted ? -(ab->mech_true + ab->mech_bias)
                                    :  (ab->mech_true + ab->mech_bias);

    /* 绝对原始值：归一化到 [0, 2π) 再映射到 [0, resolution) */
    m = fmodf(ab->mech_raw, ABS_BOARD_TWO_PI);
    if (m < 0.0f) { m += ABS_BOARD_TWO_PI; }
    ab->raw = (uint32_t)(m / ABS_BOARD_TWO_PI * (float)ab->resolution);

    ab->set_voltage_calls++;
    return 0;
}

int abs_board_get_mech_angle(void *b, float *mech_rad)
{
    AbsBoard *ab = (AbsBoard *)b;
    if ((ab == NULL) || (mech_rad == NULL)) { return 1; }
    *mech_rad = ab->mech_raw;   /* 连续机械角（board 层职责：绝对编码器连续化给校准用） */
    return 0;
}

int abs_board_wait_ms(void *b, uint32_t ms)
{
    AbsBoard *ab = (AbsBoard *)b;
    if (ab == NULL) { return 1; }
    ab->tick_us += (uint64_t)ms * 1000u;
    return 0;
}

/* ---- enc_abs 的 EncAbsHwOps 回调 ---- */
int abs_board_read_abs(void *hw, uint32_t *raw, bool *data_valid)
{
    AbsBoard *ab = (AbsBoard *)hw;
    if ((ab == NULL) || (raw == NULL) || (data_valid == NULL)) { return 1; }
    *raw = ab->raw;
    *data_valid = true;   /* 绝对编码器数据始终有效（仿真无 CRC/超时） */
    return 0;
}

uint64_t abs_board_get_tick_us(void *hw)
{
    AbsBoard *ab = (AbsBoard *)hw;
    if (ab == NULL) { return 0u; }
    return ab->tick_us;
}
