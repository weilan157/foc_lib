/*
 * foc_math.h —— FOC 坐标变换（纯数学，无 HAL / RTOS / 全局变量）
 *
 * V0.1.6 坐标系约定（写死，§3.6）：
 * - 正 Iq = 正转矩（q 轴电流符号 = 转矩符号）
 * - 编码器角度增大 = 电角度增大
 * - Clarke：ia + ib + ic = 0（星形连接，三相对称）
 * - 相序：A-B-C（电角度 0 起逆时针为正，board 可反转）
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>
#include <math.h>
#include "foc_types.h"

/* MinGW 严格 ANSI（-std=c11）下 math.h 不定义 M_PI，统一在此守卫 */
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 坐标系量 ---- */
typedef struct { float alpha; float beta; }  AlphaBeta;   /* 静止 αβ 坐标系 */
typedef struct { float d;      float q; }    Dq;          /* 旋转 dq 坐标系 */
typedef struct { float alpha_v; float beta_v; } VoltageVector; /* 电压矢量（αβ） */

/* ---- 坐标变换 ---- */

/* Clarke：三相电流 → αβ（利用 ia+ib+ic=0 简化；ic 由两相重构时置 0 输入） */
AlphaBeta foc_clarke(float ia, float ib, float ic);

/* Park：αβ → dq（电角度 theta_rad） */
Dq foc_park(float alpha, float beta, float theta_rad);

/* 逆 Park：dq → αβ（电压矢量；v_zero 为 z 轴/零序分量，通常 0） */
VoltageVector foc_inverse_park(float theta_rad, float v_zero, float v_d, float v_q);

/* 电角度：机械角 × pole_pairs + encoder_zero（编码器增大 = 电角度增大） */
float foc_calc_elec_angle(float mech_angle_rad, uint32_t pole_pairs, float encoder_zero);

/* 角度规范化到 [-pi, pi) */
float foc_wrap_pi(float a);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
