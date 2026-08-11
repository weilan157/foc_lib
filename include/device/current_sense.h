/*
 * current_sense.h —— 相电流感测（device/，V0.2 电流环，总线无关）
 *
 * 对照三家：
 *   · SimpleFOC current_sense/（Inline/Lowside/Generic）：伏安比 gain + 零点 offset 校准
 *   · ODrive CurrentSense：ADC 相电流重构 → PhaseCurrent
 *   · VESC：ADC 相电流重构 + DC 校准（mcpwm_foc_dc_cal）
 * 总线（ADC/隔离/专用 IC 等）由 board 在 HwOps 注入；本层只保留电流感测语义：
 *   raw(电压) → (raw − offset)·gain → 相电流 A，输出 SampleFrame{ia,ib,ic}。
 * 质量：任相电流超 max_current 连续 overcurrent_limit 次 → BAD（交 Fault，禁置 0）。
 */
#ifndef FOC_DEVICE_CURRENT_SENSE_H
#define FOC_DEVICE_CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>
#include "device/position_sensor.h"   /* EncQuality */
#include "runtime/sampling.h"         /* SampleFrame */

#ifdef __cplusplus
extern "C" {
#endif

/* 硬件访问抽象（总线无关：ADC / 电流检测 IC / 隔离采样由 board 注入实现） */
typedef struct {
    int      (*read_phase_raw)(void *hw, float *raw_a, float *raw_b, float *raw_c); /* 三相原始值 [V] */
    uint64_t (*get_tick_us)(void *hw);
    void     *hw;
} CurrentSenseHwOps;

/* 相电流感测设备实例（调用方持有，可静态分配） */
typedef struct {
    const CurrentSenseHwOps *hw_ops;
    /* 校准参数（cal_current 校准 / board 配置） */
    float gain_a, gain_b, gain_c;       /* 伏安比 [A/V] = 1/(Rshunt·Amp)，>0 */
    float offset_a, offset_b, offset_c; /* 零点 [V]（零电流时采样值，校准得） */
    float max_current;                  /* 过流阈值 [A]（0=禁用检测） */
    uint32_t overcurrent_limit;         /* 连续过流多少次 → BAD（防抖；0 视为 1） */

    /* 内部状态（外部只读） */
    float      last_ia, last_ib, last_ic;  /* 最近 GOOD 相电流 [A] */
    uint32_t   overcurrent_count;
    uint32_t   read_count;                 /* 总读取次数 */
    uint32_t   err_count_total;            /* 累计失效次数（错误率） */
    float      err_rate;
    EncQuality quality;
} CurrentSenseCtx;

/* 采样：读三相原始 → 零点/增益校准 → SampleFrame（电流环输入）。
 * 过流连续超限 → quality=BAD 返回 FOC_ERROR（交 Fault，禁止置 0）；
 * 读取失败 → BAD。正常 → GOOD，返回 FOC_OK。 */
int current_sense_reconstruct(void *ctx, SampleFrame *sf);
int current_sense_init(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_CURRENT_SENSE_H */
