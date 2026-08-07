/*
 * test_pipeline.c —— motor_fast_step + motor_slow_step 全链路（§25）
 * 验证：生命周期、VELOCITY 闭环收敛、GateDriver FastFault、Encoder BAD。
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

/* 注入 BAD 编码器反馈的 HwAdapterOps（覆盖 sensor_get_feedback） */
static int bad_sensor_get_feedback(void *ctx, EncoderFeedback *fb)
{
    (void)ctx;
    fb->mech_angle_rad = 0.0f;
    fb->velocity       = 0.0f;
    fb->revolution     = 0;
    fb->quality        = ENC_QUALITY_BAD;
    return FOC_OK;
}

static void run_closed_loop(MotorRuntime *rt, uint32_t steps, uint32_t slow_div)
{
    TimeBase fast_tb, slow_tb;
    uint64_t now = 0u;
    uint32_t i;

    timebase_init(&fast_tb, 0u);
    timebase_init(&slow_tb, 0u);

    for (i = 0u; i < steps; i++) {
        now += 50u;                                   /* 20kHz */
        timebase_update(&fast_tb, now, 0.001f);
        motor_fast_step(rt, &fast_tb);

        if ((i % slow_div) == 0u) {                   /* ~1kHz */
            timebase_update(&slow_tb, now, 0.002f);
            motor_slow_step(rt, &slow_tb);
        }
    }
}

int main(void)
{
    MotorRuntime rt;
    FaultReg fault;
    MotorModel model;
    SimHwCtx hw;
    PidControllerCtx pctx;
    Telemetry tel;

    /* 生命周期：INIT→SELF_TEST→CALIBRATION→READY→RUNNING */
    motor_init(&rt, 0u, &g_scfg, &hw_adapter_sim_ops, &hw,
               &pid_controller_ops, &pctx, NULL, NULL);
    rt.fault = &fault;
    rt.tel   = &tel;
    rt.safety.fault = &fault;
    fault_init(&fault);
    motor_load_config(&rt, &g_rcfg);

    motor_model_init(&model, &g_scfg);
    sim_hw_init(&hw, &model);

    CHECK(motor_self_test(&rt) == 0);
    CHECK(rt.state == FOC_STATE_CALIBRATION);
    CHECK(motor_calibrate(&rt) == 0);
    CHECK(rt.state == FOC_STATE_READY);
    CHECK(motor_enable(&rt) == 0);
    CHECK(rt.state == FOC_STATE_RUNNING);

    /* enable 必须从 READY：RUNNING 时再 enable 应失败 */
    CHECK(motor_enable(&rt) != 0);

    /* VELOCITY 闭环：目标 5 rad/s */
    {
        MotorCommand cmd = { .target = 5.0f, .mode = CTRL_MODE_VELOCITY, .sequence = 1u };
        CHECK(command_buffer_write(&rt.cmd_buf, &cmd) == 0);
    }

    run_closed_loop(&rt, 40000u, 20u);   /* 2s @20kHz */

    CHECK(rt.state == FOC_STATE_RUNNING);
    CHECK(hw.set_output_calls > 0u);
    CHECK_NEAR(model.mech_vel, 5.0f, 0.5f);   /* 速度收敛到目标 */

    /* GateDriver FastFault：置位 → 安全关断 */
    {
        hw.fast_fault_flags = 1u;
        motor_fast_step(&rt, &(TimeBase){ .dt = 0.00005f, .timestamp_us = 0u });
        CHECK(rt.state == FOC_STATE_FAULT);
        CHECK(fault_is_set(&fault, FAULT_GATE_DRIVER));
        CHECK(fault.latched != 0u);
    }

    /* Encoder BAD：quality=BAD → FAULT_ENCODER_QUALITY（禁止置 angle=0） */
    {
        HwAdapterOps bad_ops = hw_adapter_sim_ops;   /* 复制，仅覆盖 sensor_get_feedback */
        MotorRuntime rt2;
        FaultReg fault2;
        MotorModel model2;
        SimHwCtx hw2;
        PidControllerCtx pctx2;
        Telemetry tel2;

        bad_ops.sensor_get_feedback = bad_sensor_get_feedback;

        motor_init(&rt2, 1u, &g_scfg, &bad_ops, &hw2, &pid_controller_ops, &pctx2, NULL, NULL);
        rt2.fault = &fault2;
        rt2.tel   = &tel2;
        rt2.safety.fault = &fault2;
        fault_init(&fault2);
        motor_load_config(&rt2, &g_rcfg);

        motor_model_init(&model2, &g_scfg);
        sim_hw_init(&hw2, &model2);

        (void)motor_self_test(&rt2);
        (void)motor_calibrate(&rt2);
        (void)motor_enable(&rt2);

        motor_fast_step(&rt2, &(TimeBase){ .dt = 0.00005f, .timestamp_us = 0u });
        CHECK(rt2.state == FOC_STATE_FAULT);
        CHECK(fault_is_set(&fault2, FAULT_ENCODER_QUALITY));
    }

    TEST_REPORT();
}
