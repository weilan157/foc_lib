/*
 * config.h —— 参数体系唯一来源（一个定义，一个引用源；V0.1.6 冻结）
 *
 * 文档引用： #include "config.h"
 * - MotorStaticConfig   不可运行修改（硬件属性 + cap + 默认 limits）
 * - MotorCapability     电机能力包络（Limiter 读取）
 * - CommandLimitTable   按模式限幅表（limit[CTRL_MODE_TORQUE/VELOCITY/POSITION]）
 * - MotorRuntimeConfig  运行期在线可改，必须经 ConfigSnapshot 生效
 * - MotorCalibration    校准自动产生（只读）
 */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include <stdint.h>
#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 限幅区间 ---- */
typedef struct {
    float max;
    float min;
} Limit;

/* ---- 控制模式（V0.1.6 收口：唯一枚举，CommandLimitTable 索引 与 MotorCommand.mode 共用；
       CommandModeIdx / MODE_* 已合并，消除"同名不同枚举"的类型陷阱） ---- */
typedef enum {
    CTRL_MODE_TORQUE = 0,   /* 力矩 / 电压指令 */
    CTRL_MODE_VELOCITY,     /* 速度指令 [rad/s] */
    CTRL_MODE_POSITION,     /* 位置指令 [rad] */
    CTRL_MODE_MAX
} ControlMode;

/* ---- 按模式限幅表（必修 5，冻结）：弃 command_limit，改 limit[CTRL_MODE_*] ---- */
typedef struct {
    Limit limit[CTRL_MODE_MAX];   /* [CTRL_MODE_TORQUE] / [CTRL_MODE_VELOCITY] / [CTRL_MODE_POSITION] */
} CommandLimitTable;

/* ---- 电机能力包络（Limiter 使用，Controller 不知电机规格） ---- */
typedef struct {
    float max_voltage;        /* [V] */
    float max_current;        /* [A] */
    float max_speed;          /* [rad/s] */
    float max_torque;         /* [Nm] */
} MotorCapability;            /* 小关节 24V/3A；机械臂 48V/20A → 只改这里 */

/* ---- 不可运行修改：硬件属性 + 能力包络 + 默认限幅表 ---- */
typedef struct {
    uint32_t pole_pairs;
    uint32_t encoder_type;    /* ABZ / SPI_ABS / …（非运行修改，参与参数迁移） */
    float    phase_resistance;   /* [Ω] */
    float    phase_inductance;   /* [H] */
    float    kv_rating;          /* [rpm/V] */
    MotorCapability   cap;        /* 能力包络（Limiter 读取，唯一来源） */
    CommandLimitTable limits;     /* 出厂默认限幅表（启动加载；运行期生效以 ConfigSnapshot 内为准） */
} MotorStaticConfig;

/* ---- 运行时可在线修改：必须经 ConfigSnapshot 生效 ---- */
typedef struct {
    CommandLimitTable limits;     /* 运行期生效的限幅表（初始复制自 scfg->limits，在线可改） */
    /* PID 增益（V0.1 预留；V0.2 细化） */
    float kp;
    float ki;
    float kd;
} MotorRuntimeConfig;

/* ---- 校准自动产生（只读） ---- */
typedef struct {
    float    encoder_zero;       /* [rad] 编码器零点/方向对齐（= encoder_offset） */
    float    current_offset[3];  /* [A] */
    uint32_t phase_order;        /* A-B-C 默认；board 反转用（0=ABC,1=ACB） */
} MotorCalibration;

/* 限幅：x 夹在 [lim.min, lim.max]，并保证 max>=min */
float foc_limit(float x, const Limit *lim);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONFIG_H */
