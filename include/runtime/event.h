/*
 * event.h —— 事件系统（§16.4，冻结）
 * SPSC 环形缓冲（单写者）：V0.1 由 Slow Loop / Service 统一 event_publish；
 * 禁止多写者无保护共享环形缓冲。
 */
#ifndef FOC_RUNTIME_EVENT_H
#define FOC_RUNTIME_EVENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EVENT_ENCODER_READY = 0,
    EVENT_CALIBRATION_DONE,
    EVENT_MODE_CHANGED,
    EVENT_OVERTEMP_WARNING,
    EVENT_OVERVOLTAGE,
    EVENT_PARAM_CHANGED,
    EVENT_FAULT_RAISED,
    EVENT_SAFETY_ESTOP,
    EVENT_SAFETY_STO,
    EVENT_MAX
} EventType;

typedef struct {
    EventType  type;
    uint32_t   source;        /* motor_id 或 0xFFFFFFFF（全局） */
    uint64_t   timestamp_us;
    uint32_t   data;          /* 附加数据（如 DTC） */
} FocEvent;

#define EVENT_QUEUE_CAPACITY 16u   /* 2^n，便于取模掩码 */
#define EVENT_QUEUE_MASK     (EVENT_QUEUE_CAPACITY - 1u)

typedef struct {
    FocEvent         data[EVENT_QUEUE_CAPACITY];
    volatile uint32_t head;   /* 写指针（单写者） */
    volatile uint32_t tail;   /* 读指针（单读者） */
} EventQueue;

void event_queue_init(EventQueue *q);
bool event_publish(EventQueue *q, const FocEvent *ev);  /* 仅入队，非阻塞；满 → 丢弃并返回 false */
bool event_poll(EventQueue *q, FocEvent *out);          /* Service / 诊断 消费 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_EVENT_H */
