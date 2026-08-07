/*
 * command_buffer.c —— CommandBuffer 无锁双缓冲（§11，冻结）
 * 内存序：写非活动槽 → 写后屏障（release）→ swap index（32 位原子对齐写）。
 * read 保持型：out 总是返回最近有效命令；返回 bool = 自上次读是否有新命令。
 */
#include "runtime/command.h"
#include "foc_types.h"

#include <stdatomic.h>

void command_buffer_init(CommandBuffer *cb)
{
    if (cb == NULL) { return; }
    cb->index      = 0u;
    cb->last_index = 0u;
    cb->data[0].target   = 0.0f;
    cb->data[0].mode     = CTRL_MODE_VELOCITY;
    cb->data[0].sequence = 0u;
    cb->data[1] = cb->data[0];
}

int command_buffer_write(CommandBuffer *cb, const MotorCommand *cmd)
{
    uint32_t active;
    uint32_t inactive;

    if (cb == NULL || cmd == NULL) { return FOC_ERROR; }

    active   = cb->index;
    inactive = active ^ 1u;

    cb->data[inactive] = *cmd;                    /* 写非活动槽 */
    atomic_thread_fence(memory_order_release);    /* 写后内存屏障（嵌入式可 __DMB()） */
    cb->index = inactive;                         /* swap index（单写者原子写） */

    return FOC_OK;
}

bool command_buffer_read(CommandBuffer *cb, MotorCommand *out)
{
    uint32_t active;
    bool     is_new;

    if (cb == NULL || out == NULL) { return false; }

    active = cb->index;                           /* 读 index */
    *out   = cb->data[active];                    /* 读活动槽快照（保持型） */

    is_new = (active != cb->last_index);          /* 自上次读是否有新命令 */
    cb->last_index = active;
    return is_new;
}

int command_buffer_set_mode(CommandBuffer *cb, ControlMode mode)
{
    MotorCommand cmd;

    if (cb == NULL) { return FOC_ERROR; }

    command_buffer_read(cb, &cmd);                /* 取最近命令（不消耗新标志） */
    cmd.mode = mode;
    return command_buffer_write(cb, &cmd);
}

ControlMode command_buffer_get_mode(const CommandBuffer *cb)
{
    uint32_t active;

    if (cb == NULL) { return CTRL_MODE_VELOCITY; }
    active = cb->index;
    return cb->data[active].mode;
}
