/*
 * cal_current.h —— 电流零点校准插件（CalibrationOps.current 实现）
 *
 * 对照三家：
 *   · SimpleFOC InlineCurrentSense::calibrateOffsets()：零电流时读 ADC 均值 → offset
 *   · VESC mcpwm_foc_dc_cal()：零电压采样校准偏移
 *   · ODrive current_calibration：零点 + 增益
 * 流程：施加零电压（电机不转）→ 稳定 → 采样三相原始值 N 轮 → 均值 = 零点 offset。
 * 契约：结果写入本 ctx 的 offset_* 输出字段，由 board 在 motor_calibrate 返回后
 *       回填 CurrentSenseCtx.offset_* 与 MotorCalibration.current_offset[3]。
 */
#ifndef FOC_CONTROL_CAL_CURRENT_H
#define FOC_CONTROL_CAL_CURRENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 依赖注入（board 提供） */
    int  (*set_zero_voltage)(void *hw);   /* 施加零电压（PWM 全关/中心零矢量） */
    int  (*read_phase_raw)(void *hw, float *raw_a, float *raw_b, float *raw_c); /* 三相原始值 [V] */
    int  (*wait_ms)(void *hw, uint32_t ms);
    void *hw;

    /* 配置 */
    uint32_t rounds;      /* 采样轮数（0 → 默认 64） */
    uint32_t settle_ms;   /* 施加零电压后稳定等待 [ms]（0 → 默认 50） */

    /* 输出（电流零点 [V]） */
    float offset_a;
    float offset_b;
    float offset_c;
} CalCurrentCtx;

/* CalibrationOps.current 实现（ctx = CalCurrentCtx*） */
int cal_current_offset(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_CAL_CURRENT_H */
