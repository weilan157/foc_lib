/*
 * abz_board.h —— 仿真 ABZ 板（tests/simulation，§8 board 层的最小仿真）
 *
 * 桥接：把 cal_abi_align / cal_abi_find_index 的"施加电压"接到一个理想电机模型
 *       （转子瞬时跟随施加电角度），并驱动一个仿真 ABZ 编码器（count/index/时基）。
 * 用途：PC 上端到端验证 ABZ 三件套（找零 + 方向 + 对齐），不依赖真实硬件。
 * 仅用于测试/仿真；不参与目标固件。
 */
#ifndef FOC_TEST_SIMULATION_ABZ_BOARD_H
#define FOC_TEST_SIMULATION_ABZ_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pole_pairs;     /* 极对数 */
    float    mech_bias;      /* 编码器零点偏移 [rad]（模拟真实电机编码器未对齐） */
    bool     enc_inverted;   /* 编码器方向反向（模拟硬件接反/安装方向） */
    uint32_t cpr;            /* ABZ 每转计数 */

    /* 仿真状态 */
    float    theta_el;       /* 当前施加电角度 [rad] */
    float    mech_raw;       /* 编码器原始机械角读数 [rad]（含零偏/方向，未校正） */
    int32_t  abz_count;      /* 编码器计数（给 enc_abi read_count） */
    bool     index_pulse;    /* index 触发（编码器读数跨整圈边界，读后清除） */
    uint64_t tick_us;        /* 时基（给 enc_abi get_tick_us） */
    int32_t  last_rev;       /* 上一整圈数（index 检测） */
    float    prev_mech_raw;  /* 上一次读数（步进检测：仅连续步进跨边界才触发 index） */
    uint32_t set_voltage_calls;
} AbzBoard;

void abz_board_init(AbzBoard *b, uint32_t pole_pairs, float mech_bias, bool enc_inverted, uint32_t cpr);

/* 校准插件注入回调 */
int  abz_board_set_elec_voltage(void *b, float theta_el_rad, float vd, float vq);
int  abz_board_is_index_found(void *b, bool *found);
int  abz_board_get_mech_angle(void *b, float *mech_rad);
int  abz_board_wait_ms(void *b, uint32_t ms);

/* enc_abi 的 EncAbiHwOps 回调（hw 参数为 AbzBoard*） */
int      abz_board_read_count(void *hw, int32_t *count);
bool     abz_board_read_index(void *hw);
uint64_t abz_board_get_tick_us(void *hw);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TEST_SIMULATION_ABZ_BOARD_H */
