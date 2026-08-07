/*
 * motor_model.h —— 仿真电机模型（§7.4 / §25，冻结）
 * 电压 → 电机模型 → 位置/速度反馈，使 FOC 在 PC 上闭环跑。
 * 仅用于测试/仿真；不参与目标固件。
 */
#ifndef FOC_TEST_SIMULATION_MOTOR_MODEL_H
#define FOC_TEST_SIMULATION_MOTOR_MODEL_H

#include <stdint.h>
#include "foc/foc_math.h"
#include "foc/config.h"
#include "device/position_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float R;            /* 相电阻 [Ω] */
    float L;            /* 相电感 [H]（V0.1 简化：稳态忽略） */
    float ke;           /* 反电动势系数 [V·s/rad]（每相） */
    float kt;           /* 转矩系数 [N·m/A] */
    float pole_pairs;
    float J;            /* 转动惯量 [kg·m²] */
    float b;            /* 粘滞摩擦 [N·m·s/rad] */
    float vbus;         /* 母线电压 [V]（用于电压换算） */

    /* 状态 */
    float mech_angle;   /* [rad] */
    float mech_vel;     /* [rad/s] */
    float iq;           /* [A] 仿真内部 q 轴电流 */
    uint64_t timestamp_us;
    uint32_t cycle;
} MotorModel;

void motor_model_init(MotorModel *m, const MotorStaticConfig *scfg);
int  motor_model_step(MotorModel *m, const VoltageVector *vv, float dt);
void motor_model_get_feedback(const MotorModel *m, EncoderFeedback *fb);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TEST_SIMULATION_MOTOR_MODEL_H */
