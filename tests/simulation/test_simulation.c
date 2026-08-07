/*
 * test_simulation.c —— motor_model + FOC 闭环收敛（§25）
 * 直接模型验证 + 通过 motor_runtime 的闭环收敛。
 */
#include "core/motor_runtime.h"
#include "control/pid_controller.h"
#include "simulation/hw_adapter_sim.h"
#include "simulation/motor_model.h"
#include "foc_types.h"
#include "../test_assert.h"

static MotorStaticConfig g_scfg = {
    .pole_pairs = 7u,
    .encoder_type = 0u,
    .phase_resistance = 1.0f,
    .phase_inductance = 0.0001f,
    .kv_rating = 0.0f,
    .cap = { .max_voltage = 12.0f, .max_current = 3.0f, .max_speed = 50.0f, .max_torque = 0.5f },
};

static MotorRuntimeConfig g_rcfg = {
    .limits.limit[CTRL_MODE_TORQUE]   = { .max = 12.0f, .min = -12.0f },
    .limits.limit[CTRL_MODE_VELOCITY] = { .max = 50.0f, .min = -50.0f },
    .limits.limit[CTRL_MODE_POSITION] = { .max = 6.28f, .min = -6.28f },
    .kp = 0.5f, .ki = 2.0f, .kd = 0.0f,
};

int main(void)
{
    /* 1) 直接模型：恒定 αβ 电压 → 速度上升 */
    {
        MotorModel m;
        MotorStaticConfig scfg = g_scfg;
        VoltageVector vv = { .alpha_v = 0.0f, .beta_v = 1.0f };  /* 初始电角度 0：vq = beta，产生 q 轴转矩 */
        uint32_t i;

        motor_model_init(&m, &scfg);
        for (i = 0u; i < 2000u; i++) {
            (void)motor_model_step(&m, &vv, 0.00005f);
        }
        CHECK(m.mech_vel > 0.0f);          /* 电压驱动产生速度 */
        CHECK(m.mech_vel < 50.0f);         /* 被反电势/摩擦限速，未发散 */
    }

    /* 2) 通过 motor_runtime 闭环：POSITION 目标 1.0 rad 收敛 */
    {
        MotorRuntime rt;
        FaultReg fault;
        MotorModel model;
        SimHwCtx hw;
        PidControllerCtx pctx;
        Telemetry tel;
        TimeBase fast_tb, slow_tb;
        uint64_t now = 0u;
        uint32_t i;
        FastFeedback fb;

        motor_init(&rt, 0u, &g_scfg, &hw_adapter_sim_ops, &hw,
                   &pid_controller_ops, &pctx, NULL, NULL);
        rt.fault = &fault;
        rt.tel   = &tel;
        rt.safety.fault = &fault;
        fault_init(&fault);
        motor_load_config(&rt, &g_rcfg);

        motor_model_init(&model, &g_scfg);
        sim_hw_init(&hw, &model);

        (void)motor_self_test(&rt);
        (void)motor_calibrate(&rt);
        (void)motor_enable(&rt);

        {
            MotorCommand cmd = { .target = 1.0f, .mode = CTRL_MODE_POSITION, .sequence = 1u };
            (void)command_buffer_write(&rt.cmd_buf, &cmd);
        }

        timebase_init(&fast_tb, 0u);
        timebase_init(&slow_tb, 0u);

        for (i = 0u; i < 100000u; i++) {   /* 5s @20kHz */
            now += 50u;
            timebase_update(&fast_tb, now, 0.001f);
            motor_fast_step(&rt, &fast_tb);
            if ((i % 20u) == 0u) {
                timebase_update(&slow_tb, now, 0.002f);
                motor_slow_step(&rt, &slow_tb);
            }
        }

        (void)feedback_buffer_read(&rt.fb_buf, &fb);
        CHECK_NEAR(fb.mech_angle_rad, 1.0f, 0.05f);   /* 位置收敛到目标 */
        CHECK(rt.state == FOC_STATE_RUNNING);
    }

    TEST_REPORT();
}
