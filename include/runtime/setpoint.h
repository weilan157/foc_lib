/*
 * setpoint.h —— Slow(1kHz) → Fast(20kHz) 设定值缓冲（§11，冻结）
 * 单写（Slow）单读（Fast），无锁双缓冲。
 */
#ifndef FOC_RUNTIME_SETPOINT_H
#define FOC_RUNTIME_SETPOINT_H

#include <stdbool.h>
#include <stdint.h>
#include "control/controller.h"   /* ControlSetpoint */

#ifdef __cplusplus
extern "C" {
#endif

/* 双缓冲（单写单读，无锁）。index 必须 32 位对齐访存（配合 volatile）。 */
typedef struct {
    ControlSetpoint data[2];
    volatile uint32_t index;
    uint32_t        last_index;   /* 读者私有（单读者）：检测"自上次读是否有新设定值" */
} SetpointBuffer;

void setpoint_write(SetpointBuffer *sb, const ControlSetpoint *sp);  /* Slow 写 */
bool setpoint_read(SetpointBuffer *sb, ControlSetpoint *out);        /* Fast 读 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_SETPOINT_H */
