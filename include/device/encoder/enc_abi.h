/*
 * enc_abi.h —— ABZ 增量编码器（device/encoder/，§7.2 实现）
 *
 * V0.1.6 实现阶段 —— 支持增量编码器
 * - 通过注入的 EncAbiHwOps 访问硬件（board 注入真实 HAL 回调；PC 测试注入 fake）。
 * - 计数 → 机械角：angle = 2π · total_count / cpr；圈数 revolution 由累计计数整除 cpr 得。
 * - 多圈：内部累计（int64），支持正反转与硬件 mod 计数环绕（delta 修正 ±cpr/2）。
 * - 零位：
 *   · use_index=true  ：找零（index 引脚高电平 → total_count 归零），未找到前 quality=STALE（降级）。
 *   · use_index=false ：上电即相对位置累计（配合上层 encoder_zero 对齐做绝对）。
 * - 速度：一阶差分（dt 由注入时基 get_tick_us 测得；异常 dt 过滤）。
 * - 与 PositionSensorOps 统一接口，供 HardwareAdapter 使用。
 */
#ifndef FOC_DEVICE_ENCODER_ENC_ABI_H
#define FOC_DEVICE_ENCODER_ENC_ABI_H

#include <stdbool.h>
#include <stdint.h>
#include "device/position_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 硬件访问抽象（board 注入） */
typedef struct {
    int      (*read_count)(void *hw, int32_t *count);  /* 读增量计数（含硬件方向翻转） */
    bool     (*read_index)(void *hw);                  /* index 引脚电平（use_index 时用） */
    uint64_t (*get_tick_us)(void *hw);                 /* 微秒时基（速度微分） */
    void     *hw;
} EncAbiHwOps;

/* ABZ 设备实例（调用方持有，可静态分配） */
typedef struct {
    const EncAbiHwOps *hw_ops;
    uint32_t cpr;            /* 每转计数（如 4096），>0 */
    bool     use_index;      /* true：index 找零；false：相对累计 */
    bool     inverted;       /* 计数方向反向（校准得，phase 校准写入） */

    /* 可选 PLL 位置/速度估计（ODrive 式临界阻尼；默认关闭 → 裸计数/一阶差分） */
    bool     use_pll;             /* true：位置/速度走 PLL 平滑（需 pll_bandwidth_hz>0） */
    float    pll_bandwidth_hz;    /* PLL 带宽 [Hz]（use_pll 时） */

    /* 内部状态（外部只读） */
    int64_t    total_count;  /* 累计计数（含圈数，可负） */
    int32_t    last_count;
    int32_t    half_cpr;     /* 预计算 cpr/2（环绕修正，避免每周期整数除法） */
    float      scale;        /* 预计算 2π/cpr（角度换算，乘法替代浮点除法） */
    float      last_angle_rad;
    float      velocity;     /* [rad/s]（一阶差分；use_pll 时不用） */
    uint64_t   last_tick_us;
    bool       index_found;
    bool       first;
    EncQuality quality;

    /* PLL 状态（use_pll 时） */
    float      pll_pos;      /* PLL 平滑位置 [rad] */
    float      pll_vel;      /* PLL 速度 [rad/s] */
    float      pll_kp;       /* 预计算增益 [1/s] = 2·bw */
    float      pll_ki;       /* 预计算增益 [1/s²] = 0.25·kp² */
    bool       pll_first;
} EncAbiCtx;

/* PositionSensorOps 实现（enc_abi_ops.init/update/get_feedback 用 ctx=EncAbiCtx*） */
int enc_abi_init(void *ctx);
int enc_abi_update(void *ctx);
int enc_abi_get_feedback(void *ctx, EncoderFeedback *fb);
extern const PositionSensorOps enc_abi_ops;

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_ENCODER_ENC_ABI_H */
