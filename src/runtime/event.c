/*
 * event.c —— Event 系统 SPSC 环形缓冲（§16.4，冻结）
 * 单写者（Slow/Service 统一 publish）单读者（Service/诊断 poll）；禁止多写者。
 */
#include "runtime/event.h"

void event_queue_init(EventQueue *q)
{
    if (q == NULL) { return; }
    q->head = 0u;
    q->tail = 0u;
}

bool event_publish(EventQueue *q, const FocEvent *ev)
{
    uint32_t head;
    uint32_t next;

    if (q == NULL || ev == NULL) { return false; }

    head = q->head;
    next = (head + 1u) & EVENT_QUEUE_MASK;

    if (next == q->tail) { return false; }   /* 满：丢弃 */

    q->data[head] = *ev;
    q->head = next;                          /* 单写者推进 */
    return true;
}

bool event_poll(EventQueue *q, FocEvent *out)
{
    uint32_t tail;

    if (q == NULL || out == NULL) { return false; }

    tail = q->tail;
    if (tail == q->head) { return false; }   /* 空 */

    *out = q->data[tail];
    q->tail = (tail + 1u) & EVENT_QUEUE_MASK; /* 单读者推进 */
    return true;
}
