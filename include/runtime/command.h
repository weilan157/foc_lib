/*
 * command.h —— 命令唯一来源（§11，冻结）
 * Service(慢) → CommandBuffer → Slow Loop(1kHz)。
 * 无锁双缓冲 + 显式内存序；read 为保持型语义（无新命令 → out=最近值，返回 false）。
 */
#ifndef FOC_RUNTIME_COMMAND_H
#define FOC_RUNTIME_COMMAND_H

#include <stdbool.h>
#include <stdint.h>
#include "foc/config.h"   /* ControlMode 唯一定义在 foc/config.h */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float       target;     /* rad / rad/s / V（TORQUE 模式=电压指令 voltage_sp） */
    ControlMode mode;
    uint32_t    sequence;
} MotorCommand;

/* 双缓冲（单写单读，无锁）。index 必须 32 位对齐访存（配合 volatile）。 */
typedef struct {
    MotorCommand   data[2];
    volatile uint32_t index;
    uint32_t       last_index;   /* 读者私有（单读者）：检测"自上次读是否有新命令" */
} CommandBuffer;

void        command_buffer_init(CommandBuffer *cb);
int         command_buffer_write(CommandBuffer *cb, const MotorCommand *cmd);
bool        command_buffer_read(CommandBuffer *cb, MotorCommand *out);   /* 保持型：无新命令→out=最近值，返回 false */
int         command_buffer_set_mode(CommandBuffer *cb, ControlMode mode);
ControlMode command_buffer_get_mode(const CommandBuffer *cb);
float       command_buffer_get_target(const CommandBuffer *cb);           /* 非消耗读目标（不碰 last_index，供遥测/诊断） */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_COMMAND_H */
