/*
 * cal_abi_find_index.c —— ABZ 找零驱动实现
 *
 * 流程：施加开环旋转电压（电角度连续递增），每步推进编码器并查询 index，
 *       直到 enc_abi.index_found；超圈则失败。总电圈数 = max_mech_turns · pole_pairs。
 */
#include "control/cal_abi_find_index.h"
#include "foc_types.h"

#define CAL_FI_TWO_PI      (6.283185307179586f)
#define CAL_FI_WAIT_RESET_MS (10u)

int cal_abi_find_index(void *ctx)
{
    CalAbiFindIndexCtx *c = (CalAbiFindIndexCtx *)ctx;
    uint32_t steps_per_turn;
    uint32_t total_elec_turns;
    uint32_t i;
    bool found = false;

    if ((c == NULL) || (c->set_elec_voltage == NULL) || (c->sensor_update == NULL) ||
        (c->is_index_found == NULL) || (c->wait_ms == NULL)) {
        return FOC_ERROR;
    }
    if ((c->pole_pairs == 0u) || (c->align_voltage <= 0.0f)) {
        return FOC_ERROR;
    }

    /* 缺省值 */
    if (c->steps_per_turn == 0u) { c->steps_per_turn = 500u; }
    if (c->step_ms == 0u)        { c->step_ms = 5u; }
    if (c->max_mech_turns == 0u) { c->max_mech_turns = 3u; }

    steps_per_turn = c->steps_per_turn;
    total_elec_turns = c->max_mech_turns * c->pole_pairs;

    /* 施加开环旋转电压（电角度 0 → total_elec_turns·2π 连续递增，多圈），每步推进编码器并查 index */
    for (i = 0u; i < (total_elec_turns * steps_per_turn); i++) {
        float theta = (float)i * CAL_FI_TWO_PI / (float)steps_per_turn;   /* 不取模：电角度跨圈连续增 */
        (void)c->set_elec_voltage(c->hw, theta, 0.0f, c->align_voltage);
        if (c->sensor_update(c->hw) != FOC_OK) { return FOC_ERROR; }
        (void)c->wait_ms(c->hw, c->step_ms);
        if (c->is_index_found(c->hw, &found) != FOC_OK) { return FOC_ERROR; }
        if (found) { break; }
    }

    /* 去磁复位 */
    (void)c->set_elec_voltage(c->hw, 0.0f, 0.0f, 0.0f);
    (void)c->wait_ms(c->hw, CAL_FI_WAIT_RESET_MS);

    return found ? FOC_OK : FOC_ERROR;   /* 超圈未找到 → 失败 */
}
