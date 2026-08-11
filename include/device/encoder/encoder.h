/*
 * encoder.h —— 编码器设备聚合（device/encoder/，§3.2.1 目录）
 *
 * V0.1.6 实现阶段 —— 绝对编码器（ABS，总线无关）+ 增量编码器（ABZ）装配入口。
 * - 语义在 device：这里只做"按类型返回 PositionSensorOps"，实例化与 HAL/总线绑定在 board。
 * - EncoderType 与 foc/config.h 的 MotorStaticConfig.encoder_type（uint32_t）由 board 映射，
 *   不在冻结的 config.h 内加枚举（保持参数体系唯一来源不变）。
 */
#ifndef FOC_DEVICE_ENCODER_ENCODER_H
#define FOC_DEVICE_ENCODER_ENCODER_H

#include "device/encoder/enc_abi.h"
#include "device/encoder/enc_abs.h"
#include "device/position_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_TYPE_ABZ = 0,     /* 增量：ABZ + index（或相对累计） */
    ENCODER_TYPE_ABS,         /* 绝对：总线无关绝对编码器（SPI/I2C/UART/PWM/SSI/BiSS，带质量） */
    ENCODER_TYPE_COUNT
} EncoderType;

/* 按类型返回 PositionSensorOps（board 装配用；未知类型返回 NULL） */
const PositionSensorOps *encoder_ops_for_type(EncoderType type);

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_ENCODER_ENCODER_H */
