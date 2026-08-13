/*
 * enc_abs.h —— 绝对编码器（device/encoder/，§7.2 实现，总线无关）
 *
 * V0.1.6 实现阶段 —— 支持绝对编码器，与总线无关（bus-agnostic）。
 * - 上电即读绝对角（无需 index / 找零）；总线协议（SPI/I2C/UART/PWM/SSI/BiSS 等）
 *   完全由 board/HAL 在 HwOps 回调中实现，本设备层只保留"绝对编码器语义"：
 *     raw → 机械角、数据有效（CRC/诊断/超时）→ 质量分级、方向 inverted、
 *     多圈 revolution（可选 read_multi）、速度估计（跨圈环绕差分 / 可选 PLL）、错误率。
 * - 对照参考库：
 *   · SimpleFOC Sensor 基类 + SPI/I2C/PWM 子类 —— 协议在驱动层，语义在设备层（同构）。
 *   · VESC encoder_read_deg() + 各编码器驱动 —— 统一接口 + 错误率统计（err_rate）。
 *   · ODrive Encoder mode 选择 —— 绝对模式共享增量同一状态机。
 * - 质量分级（对照 foc_lib §4.3 语义）：
 *   · data_valid=true               → GOOD，更新角度。
 *   · data_valid=false 单次/少数     → STALE（降级运行，沿用旧角度，不立即 Fault）。
 *   · 连续 data_err_limit 次错误     → BAD（交 Fault，禁止置 angle=0）。
 * - 多圈：单圈 raw ∈ [0,resolution)；可选 read_multi 提供独立圈数（EnDat/BiSS 多圈）。
 * - 速度：单圈带符号增量（跨圈环绕修正）+ 圈数增量（read_multi）差分；可选 PLL。
 */
#ifndef FOC_DEVICE_ENCODER_ENC_ABS_H
#define FOC_DEVICE_ENCODER_ENC_ABS_H

#include <stdbool.h>
#include <stdint.h>
#include "device/position_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 硬件访问抽象（总线无关：SPI/I2C/UART/PWM/SSI/BiSS 等协议由 board/HAL 注入实现） */
typedef struct {
    int      (*read_abs)(void *hw, uint32_t *raw, bool *data_valid);  /* 读绝对原始值 + 数据有效（CRC/诊断/超时） */
    int      (*read_multi)(void *hw, int32_t *turns);                 /* 可选：多圈圈数（NULL = 单圈，turns=0） */
    uint64_t (*get_tick_us)(void *hw);                                /* 微秒时基（速度微分） */
    void     *hw;
} EncAbsHwOps;

/* 绝对编码器设备实例（调用方持有，可静态分配） */
typedef struct {
    const EncAbsHwOps *hw_ops;
    uint32_t resolution;      /* 单圈分辨率（如 16384），>0 */
    uint32_t data_err_limit;  /* 连续数据无效多少次 → BAD（防抖；0 视为 1） */
    bool     inverted;        /* 单圈角度方向反向（raw 取反到 [0,res)，相位校准写入，
                                  对照 enc_abi.inverted / SimpleFOC direction） */

    /* 可选 PLL 位置/速度估计（ODrive 式临界阻尼；默认关闭 → 裸差分） */
    bool     use_pll;             /* true：位置/速度走 PLL 平滑（需 pll_bandwidth_hz>0） */
    float    pll_bandwidth_hz;    /* PLL 带宽 [Hz]（use_pll 时） */

    /* 内部状态（外部只读） */
    uint32_t last_raw;        /* 最近一次 GOOD 的单圈原始值 [0, resolution) */
    uint32_t half_res;        /* 预计算 resolution/2（环绕修正，避免每周期整数除法） */
    float    scale;           /* 预计算 2π/resolution（角度换算，乘法替代浮点除法） */
    uint32_t data_err_count;  /* 连续无效计数 */
    uint32_t update_count;    /* 累计读取次数（含有效/无效，错误率统计用） */
    uint32_t err_count_total; /* 累计无效次数（对照 VESC encoder_get_error_rate） */
    float    err_rate;        /* 错误率 [0,1] = 无效 / 总读取 */
    int32_t  last_turn;       /* 上一圈数（read_multi 或 raw/resolution；速度跨圈差分） */
    float    last_angle_rad;  /* 最近 GOOD 的多圈连续机械角 [rad]（可超 2π） */
    float    velocity;        /* [rad/s]（一阶差分；use_pll 时不用） */
    uint64_t last_tick_us;
    bool     first;           /* 首次 update：只建基准，不产出位移 */
    EncQuality quality;

    /* PLL 状态（use_pll 时） */
    float      pll_pos;       /* PLL 平滑位置 [rad]（多圈连续） */
    float      pll_vel;       /* PLL 速度 [rad/s] */
    float      pll_kp;        /* 预计算增益 [1/s] = 2·bw */
    float      pll_ki;        /* 预计算增益 [1/s²] = 0.25·kp² */
    bool       pll_first;
} EncAbsCtx;

/* PositionSensorOps 实现（enc_abs_ops.init/update/get_feedback 用 ctx=EncAbsCtx*） */
int enc_abs_init(void *ctx);
int enc_abs_update(void *ctx);
int enc_abs_get_feedback(void *ctx, EncoderFeedback *fb);
extern const PositionSensorOps enc_abs_ops;

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_ENCODER_ENC_ABS_H */
