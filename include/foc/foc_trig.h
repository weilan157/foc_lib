/*
 * foc_trig.h —— 三角实现抽象（纯算法层，无 HAL / RTOS / 全局变量）
 *
 * V0.1.6 Architecture Baseline —— FOC 效率优化
 *
 * 背景：Park / 逆Park 每周期 4 次 sin/cos，是 FOC 最大热点。标准库 cosf/sinf
 *       的周期 / ROM / 确定性不可控，算法层禁用（§4.8.1 Fast Loop 预算）。
 *
 * 实现选择（编译期宏 FOC_TRIG_IMPL）：
 *   - FOC_TRIG_IMPL_LOOKUP（默认 0）：纯 C 查表 + 线性插值。
 *       · 257 点 float sin 表（0..π/2）+ 4 象限折叠，插值误差 ~3e-5。
 *       · 确定性（固定 cycle）、零硬件依赖、PC 单测可跑。
 *   - FOC_TRIG_IMPL_POLY（1）：VESC 式 Bhaskara 多项式近似。
 *       · 零表、更快、误差 ~1e-3；适合 ROM 极省或对精度要求低的场合。
 *   - FOC_TRIG_IMPL_CORDIC（2）：STM32G4/H7 硬件 CORDIC 后端。
 *       · 算法层只声明 extern foc_sincos_cordic()，实现放 board/hal 层
 *         （依赖规则 §5.2：算法层禁止 include hal.h，仅链接外部符号）。
 *       · 编译时 -DFOC_TRIG_IMPL=2，并在 HAL 层提供 foc_sincos_cordic()。
 *
 * 依赖规则：本文件仅纯函数声明 + 编译期宏开关，不 include 任何 HAL/RTOS。
 * 用法：FOC 主路径一次拿 sin+cos → 用 foc_sincos()（避免分两次调 sin/cos）。
 */
#ifndef FOC_TRIG_H
#define FOC_TRIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TRIG_IMPL_LOOKUP  0   /* 查表 + 线性插值（默认，纯 C） */
#define FOC_TRIG_IMPL_POLY    1   /* Bhaskara 多项式近似（纯 C，零表） */
#define FOC_TRIG_IMPL_CORDIC  2   /* 硬件 CORDIC（board/HAL 提供 foc_sincos_cordic） */

#ifndef FOC_TRIG_IMPL
#define FOC_TRIG_IMPL FOC_TRIG_IMPL_LOOKUP
#endif

#if (FOC_TRIG_IMPL == FOC_TRIG_IMPL_CORDIC)
/* 硬件后端：由 board/hal 层实现（如 STM32G4/H7 CORDIC 外设） */
void foc_sincos_cordic(float theta_rad, float *s, float *c);
#endif

/* 一次算 sin/cos（theta_rad 任意弧度，内部归约到 [0, 2π)） */
void foc_sincos(float theta_rad, float *s, float *c);

/* 便捷单值接口（FOC 主路径请用 foc_sincos，一次拿两个） */
float foc_sin(float theta_rad);
float foc_cos(float theta_rad);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRIG_H */
