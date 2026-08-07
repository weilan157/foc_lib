/*
 * foc_types.h —— FOC 基础类型与常量（纯算法层，无 HAL / RTOS / 全局变量）
 *
 * V0.1.6 Architecture Baseline（Final Freeze Approved）
 * - 本文件只放"纯类型 + 纯函数"，保证 PC 单测可用。
 * - 单位：内部一律 SI（rad / rad/s / A / V / s）。
 * - 参数体系（Limit/CommandLimitTable/MotorCapability/MotorStaticConfig/...）在 foc/config.h（唯一来源）。
 */
#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdbool.h>
#include <stddef.h>   /* NULL */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 状态码 ---- */
#define FOC_OK      0
#define FOC_ERROR   1

/* 通用夹取：x 夹在 [lo, hi] */
float foc_clampf(float x, float lo, float hi);

/* 有限性判定（NaN/Inf 防护，MISRA-C 浮点强制） */
bool foc_is_finite(float x);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TYPES_H */
