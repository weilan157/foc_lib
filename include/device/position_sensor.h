/*
 * position_sensor.h —— PositionSensor（§7.2，冻结）
 * 统一输出 EncoderFeedback + quality；禁止坏数据置 angle=0。
 */
#ifndef FOC_DEVICE_POSITION_SENSOR_H
#define FOC_DEVICE_POSITION_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENC_QUALITY_GOOD = 0,
    ENC_QUALITY_BAD,        /* CRC 错误 / 超时 / 信号异常 */
    ENC_QUALITY_STALE,      /* 数据过期 */
} EncQuality;

typedef struct {
    float       mech_angle_rad;   /* 机械角 [rad] */
    int32_t     revolution;       /* 圈数（ABZ 累计；SPI 绝对为 0/当前圈） */
    float       velocity;         /* [rad/s] */
    EncQuality  quality;          /* 重要：BAD → 交 Fault，禁止置 angle=0 */
} EncoderFeedback;

typedef struct {
    int (*init)(void *ctx);
    int (*update)(void *ctx);
    int (*get_feedback)(void *ctx, EncoderFeedback *fb);
} PositionSensorOps;

typedef struct { const PositionSensorOps *ops; void *ctx; } PositionSensor;

#ifdef __cplusplus
}
#endif

#endif /* FOC_DEVICE_POSITION_SENSOR_H */
