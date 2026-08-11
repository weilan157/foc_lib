/*
 * abs_board.h —— 仿真绝对编码器板（tests/simulation，§8 board 层的最小仿真）
 *
 * 桥接：把 cal_abi_align 的"施加电压"接到理想电机模型（转子瞬时跟随施加电角度），
 *       并驱动一个仿真绝对编码器（raw 绝对位置，无 index/找零），供 enc_abs 读取。
 * 用途：PC 上端到端验证"绝对编码器相位校准"（方向 + 对齐 + encoder_zero 反推 + 电角度闭环），
 *       不依赖真实硬件、与总线无关（board 模拟任意总线读出 raw）。
 * 仅用于测试/仿真；不参与目标固件。
 */
#ifndef FOC_TEST_SIMULATION_ABS_BOARD_H
#define FOC_TEST_SIMULATION_ABS_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pole_pairs;     /* 极对数 */
    float    mech_bias;      /* 编码器零点偏移 [rad]（模拟编码器未对齐） */
    bool     enc_inverted;   /* 编码器方向反向（模拟硬件接反/安装方向） */
    uint32_t resolution;     /* 单圈分辨率（如 16384） */

    /* 仿真状态 */
    float    theta_el;       /* 当前施加电角度 [rad] */
    float    mech_true;      /* 真实机械角 [rad]（连续） */
    float    mech_raw;       /* 编码器连续机械读数 [rad]（含 bias/方向，连续，给校准） */
    uint32_t raw;            /* 绝对原始值 [0,resolution)（给 enc_abs read_abs） */
    uint64_t tick_us;        /* 时基（给 enc_abs get_tick_us） */
    uint32_t set_voltage_calls;
} AbsBoard;

void abs_board_init(AbsBoard *b, uint32_t pole_pairs, float mech_bias,
                    bool enc_inverted, uint32_t resolution);

/* 校准插件注入回调 */
int  abs_board_set_elec_voltage(void *b, float theta_el_rad, float vd, float vq);
int  abs_board_get_mech_angle(void *b, float *mech_rad);   /* 连续机械角（board 负责绝对编码器连续化） */
int  abs_board_wait_ms(void *b, uint32_t ms);

/* enc_abs 的 EncAbsHwOps 回调（hw 参数为 AbsBoard*） */
int      abs_board_read_abs(void *hw, uint32_t *raw, bool *data_valid);
uint64_t abs_board_get_tick_us(void *hw);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TEST_SIMULATION_ABS_BOARD_H */
