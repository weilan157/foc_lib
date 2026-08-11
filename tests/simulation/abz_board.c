/*
 * abz_board.c —— 仿真 ABZ 板实现（理想电机 + 仿真 ABZ 编码器）
 *
 * 模型假设（仅测试）：
 *   - 转子瞬时跟随施加电角度：mech_true = theta_el / pole_pairs（无惯性/无负载）
 *   - 编码器原始读数 = ±(mech_true + mech_bias)，方向由 enc_inverted 决定
 *   - index 在机械角跨整圈边界时触发（每机械转一次）
 */
#include "simulation/abz_board.h"
#include <math.h>   /* fabsf：步进阈值检测 */

#define ABZ_BOARD_TWO_PI (6.283185307179586f)
#define ABZ_BOARD_PI     (3.141592653589793f)

/* 步进阈值 [rad]：读数跳变超过半圈视为大幅瞬移/去磁跳变，不算经过 index */
#define ABZ_BOARD_STEP_RAD (3.141592653589793f)

void abz_board_init(AbzBoard *b, uint32_t pole_pairs, float mech_bias, bool enc_inverted, uint32_t cpr)
{
    if (b == NULL) { return; }
    b->pole_pairs    = (pole_pairs == 0u) ? 1u : pole_pairs;
    b->mech_bias     = mech_bias;
    b->enc_inverted  = enc_inverted;
    b->cpr           = (cpr == 0u) ? 1024u : cpr;
    b->theta_el      = 0.0f;
    b->mech_raw      = 0.0f;
    b->abz_count     = 0;
    b->index_pulse   = false;
    b->tick_us       = 0u;
    b->last_rev      = 0;
    b->prev_mech_raw = 0.0f;
    b->set_voltage_calls = 0u;
}

int abz_board_set_elec_voltage(void *b, float theta_el_rad, float vd, float vq)
{
    AbzBoard *ab = (AbzBoard *)b;
    float mech_true;
    float rev;
    int32_t rev_int;

    if (ab == NULL) { return 1; }
    (void)vd;
    (void)vq;

    ab->theta_el = theta_el_rad;
    mech_true = theta_el_rad / (float)ab->pole_pairs;

    /* 编码器原始读数（含零偏；方向按 enc_inverted 模拟硬件接线） */
    ab->mech_raw = ab->enc_inverted ? -(mech_true + ab->mech_bias) : (mech_true + ab->mech_bias);
    ab->abz_count = (int32_t)(ab->mech_raw / ABZ_BOARD_TWO_PI * (float)ab->cpr);

    /* index：编码器读数（mech_raw）跨整圈边界 → 触发一次。
       仅连续步进移动（|Δ| < 半圈）视为转子经过 index；大幅瞬移/去磁跳变
       （如 find 末尾 set(0,0,0) 从一整圈外拉回）不算经过 index。
       真实 ABZ：index 脉冲在编码器自身零位，与安装偏置无关。 */
    rev = ab->mech_raw / ABZ_BOARD_TWO_PI;
    rev_int = (int32_t)rev;
    if (rev_int != ab->last_rev) {
        if (fabsf(ab->mech_raw - ab->prev_mech_raw) < ABZ_BOARD_STEP_RAD) {
            ab->index_pulse = true;
        }
        ab->last_rev = rev_int;
    }
    ab->prev_mech_raw = ab->mech_raw;

    ab->set_voltage_calls++;
    return 0;
}

int abz_board_is_index_found(void *b, bool *found)
{
    AbzBoard *ab = (AbzBoard *)b;
    if ((ab == NULL) || (found == NULL)) { return 1; }
    *found = ab->index_pulse;
    return 0;
}

int abz_board_get_mech_angle(void *b, float *mech_rad)
{
    AbzBoard *ab = (AbzBoard *)b;
    if ((ab == NULL) || (mech_rad == NULL)) { return 1; }
    *mech_rad = ab->mech_raw;   /* 编码器原始读数（含方向/零偏） */
    return 0;
}

int abz_board_wait_ms(void *b, uint32_t ms)
{
    AbzBoard *ab = (AbzBoard *)b;
    if (ab == NULL) { return 1; }
    ab->tick_us += (uint64_t)ms * 1000u;
    return 0;
}

/* ---- enc_abi 的 EncAbiHwOps 回调 ---- */
int abz_board_read_count(void *hw, int32_t *count)
{
    AbzBoard *ab = (AbzBoard *)hw;
    if ((ab == NULL) || (count == NULL)) { return 1; }
    *count = ab->abz_count;
    return 0;
}

bool abz_board_read_index(void *hw)
{
    AbzBoard *ab = (AbzBoard *)hw;
    bool p;
    if (ab == NULL) { return false; }
    p = ab->index_pulse;
    ab->index_pulse = false;   /* 边沿触发：读后清除（模拟真实单次 index 脉冲） */
    return p;
}

uint64_t abz_board_get_tick_us(void *hw)
{
    AbzBoard *ab = (AbzBoard *)hw;
    if (ab == NULL) { return 0u; }
    return ab->tick_us;
}
