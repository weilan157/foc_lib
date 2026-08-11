/*
 * cal_abi_find_index.h —— ABZ 找零驱动（CalibrationOps.encoder 的一个实现）
 *
 * 对照 SimpleFOC absoluteZeroSearch / VESC 扫相：施加开环旋转电压，
 * 让转子匀速转圈，直到 ABZ 编码器找到 index（enc_abi.index_found = true）。
 *
 * 依赖注入（board 提供回调；测试注入 fake）：
 *   - set_elec_voltage(hw, theta_el, vd, vq)：施加旋转电压（V0.1 电压模式）
 *   - sensor_update(hw)：推进编码器状态（内部调 enc_abi_update，使计数/index 被处理）
 *   - is_index_found(hw, &found)：查 enc_abi.index_found
 *   - wait_ms(hw, ms)：阻塞等待（RTOS delay / 空转）
 *
 * 契约：找到 index → FOC_OK；超 max_mech_turns 机械圈仍未找到 → FOC_ERROR。
 */
#ifndef FOC_CONTROL_CAL_ABI_FIND_INDEX_H
#define FOC_CONTROL_CAL_ABI_FIND_INDEX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 依赖注入（board 提供） */
    int  (*set_elec_voltage)(void *hw, float theta_el_rad, float vd, float vq);
    int  (*sensor_update)(void *hw);                /* 推进编码器（每步；内部调 enc_abi_update） */
    int  (*is_index_found)(void *hw, bool *found);  /* 查 enc_abi.index_found */
    int  (*wait_ms)(void *hw, uint32_t ms);
    void *hw;

    /* 配置 */
    uint32_t pole_pairs;       /* 极对数（机械圈→电圈换算），>0 */
    float    align_voltage;    /* 旋转电压 [V]，>0 */
    uint32_t steps_per_turn;   /* 每电圈步数，默认 500 */
    uint32_t step_ms;          /* 每步等待 [ms]，默认 5 */
    uint32_t max_mech_turns;   /* 最大机械圈数（找零上限），默认 3 */
} CalAbiFindIndexCtx;

/* CalibrationOps.encoder 实现（ctx = CalAbiFindIndexCtx*） */
int cal_abi_find_index(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_CAL_ABI_FIND_INDEX_H */
