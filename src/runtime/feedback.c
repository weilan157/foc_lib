/*
 * feedback.c —— FeedbackBuffer（§4.3，必修 6，冻结）
 * Fast(20kHz) 写，Slow/Controller/诊断读。多读者 → read 不做"新数据"检测，
 * 只返回最近一致快照（保持型）。
 */
#include "runtime/feedback.h"
#include "foc_types.h"

#include <stdatomic.h>

void feedback_buffer_init(FeedbackBuffer *fb_buf)
{
    if (fb_buf == NULL) { return; }
    fb_buf->index = 0u;
    fb_buf->data[0].mech_angle_rad = 0.0f;
    fb_buf->data[0].mech_vel_radps = 0.0f;
    fb_buf->data[0].elec_angle_rad = 0.0f;
    fb_buf->data[0].quality        = FEEDBACK_STALE;
    fb_buf->data[1] = fb_buf->data[0];
}

void feedback_buffer_write(FeedbackBuffer *fb_buf, const FastFeedback *fb)
{
    uint32_t active;
    uint32_t inactive;

    if (fb_buf == NULL || fb == NULL) { return; }

    active   = fb_buf->index;
    inactive = active ^ 1u;

    fb_buf->data[inactive] = *fb;
    atomic_thread_fence(memory_order_release);
    fb_buf->index = inactive;
}

bool feedback_buffer_read(FeedbackBuffer *fb_buf, FastFeedback *out)
{
    uint32_t active;

    if (fb_buf == NULL || out == NULL) { return false; }

    active = fb_buf->index;
    *out   = fb_buf->data[active];   /* 保持型：总是返回最近一致快照 */
    return true;
}
