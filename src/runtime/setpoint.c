/*
 * setpoint.c —— SetpointBuffer 无锁双缓冲（§11，冻结）
 * Slow(1kHz) 写，Fast(20kHz) 读；保持型读（无新设定值时 out 仍为最近值）。
 */
#include "runtime/setpoint.h"
#include "foc_types.h"

#include <stdatomic.h>

void setpoint_write(SetpointBuffer *sb, const ControlSetpoint *sp)
{
    uint32_t active;
    uint32_t inactive;

    if (sb == NULL || sp == NULL) { return; }

    active   = sb->index;
    inactive = active ^ 1u;

    sb->data[inactive] = *sp;
    atomic_thread_fence(memory_order_release);
    sb->index = inactive;
}

bool setpoint_read(SetpointBuffer *sb, ControlSetpoint *out)
{
    uint32_t active;
    bool     is_new;

    if (sb == NULL || out == NULL) { return false; }

    active = sb->index;
    *out   = sb->data[active];                    /* 保持型 */

    is_new = (active != sb->last_index);
    sb->last_index = active;
    return is_new;
}
