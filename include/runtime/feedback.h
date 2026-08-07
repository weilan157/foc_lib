/*
 * feedback.h —— 反馈唯一出口（必修 6，冻结，§4.3）
 * 唯一来源：Sensor → FeedbackBuffer → Controller
 * 禁止 MotorRuntime 再存一份 angle/velocity（两份必然漂移）。
 * 算法层依赖：无（纯类型 + 双缓冲）。
 */
#ifndef FOC_RUNTIME_FEEDBACK_H
#define FOC_RUNTIME_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 反馈质量分级（V0.1.6：仅记录、不判定；V0.2 经 FaultMonitor 接入防抖判定） */
typedef enum {
    FEEDBACK_OK = 0,      /* 数据有效 */
    FEEDBACK_STALE,       /* 单次 CRC 错 / 数据过期 → 降级运行（不立即 Fault） */
    FEEDBACK_INVALID,     /* 连续错误 → 数据不可用 → 交 Fault（V0.2 赋值） */
    FEEDBACK_LIMITED,     /* 数据经限幅 / 降级处理（V0.2 赋值） */
} FeedbackQuality;

typedef struct {
    float           mech_angle_rad;   /* 机械角 [rad] */
    float           mech_vel_radps;   /* 机械角速度 [rad/s] */
    float           elec_angle_rad;   /* 电角度 [rad]（由 mech + pole_pairs + encoder_zero 派生） */
    FeedbackQuality quality;          /* 必须带 quality */
} FastFeedback;

/* 双缓冲（单写单读，无锁）。index 必须 32 位对齐访存（配合 volatile）。 */
typedef struct {
    FastFeedback   data[2];
    volatile uint32_t index;
} FeedbackBuffer;

void feedback_buffer_init(FeedbackBuffer *fb_buf);
void feedback_buffer_write(FeedbackBuffer *fb_buf, const FastFeedback *fb); /* Fast 写 */
bool feedback_buffer_read(FeedbackBuffer *fb_buf, FastFeedback *out);       /* 任意消费方读 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_FEEDBACK_H */
