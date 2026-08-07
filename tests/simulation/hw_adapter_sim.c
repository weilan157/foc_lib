/*
 * hw_adapter_sim.c —— 仿真 HardwareAdapter 实现
 * gate_set_output 里推进电机模型一步（闭环）；sensor 返回模型反馈。
 */
#include "simulation/hw_adapter_sim.h"
#include "foc_types.h"

static int sim_sensor_update(void *ctx)
{
    (void)ctx;
    return FOC_OK;   /* 反馈由模型直接读取，无需单独 update */
}

static int sim_sensor_get_feedback(void *ctx, EncoderFeedback *fb)
{
    SimHwCtx *c = (SimHwCtx *)ctx;
    if (c == NULL || c->model == NULL || fb == NULL) { return FOC_ERROR; }
    motor_model_get_feedback(c->model, fb);
    return FOC_OK;
}

static int sim_current_reconstruct(void *ctx, SampleFrame *sf)
{
    (void)ctx;
    (void)sf;
    return FOC_ERROR;   /* V0.2：电流采样 */
}

static int sim_gate_set_output(void *ctx, const VoltageVector *v)
{
    SimHwCtx *c = (SimHwCtx *)ctx;
    if (c == NULL || v == NULL) { return FOC_ERROR; }

    c->last_vv = *v;
    c->set_output_calls++;

    /* 闭环：每次 Fast Loop 写入电压即推进模型一步 */
    if (c->model != NULL) {
        (void)motor_model_step(c->model, v, c->sim_dt);
    }
    return FOC_OK;
}

static int sim_gate_fast_check(void *ctx, GateFastFault *fault)
{
    SimHwCtx *c = (SimHwCtx *)ctx;
    if (c == NULL || fault == NULL) { return FOC_ERROR; }
    fault->fault_flags = c->fast_fault_flags;
    return FOC_OK;
}

static int sim_gate_status_update(void *ctx, GateDriverStatus *st)
{
    SimHwCtx *c = (SimHwCtx *)ctx;
    if (c == NULL || st == NULL) { return FOC_ERROR; }
    *st = c->status;
    return FOC_OK;
}

const HwAdapterOps hw_adapter_sim_ops = {
    .sensor_update       = sim_sensor_update,
    .sensor_get_feedback = sim_sensor_get_feedback,
    .current_reconstruct = sim_current_reconstruct,
    .gate_set_output     = sim_gate_set_output,
    .gate_fast_check     = sim_gate_fast_check,
    .gate_status_update  = sim_gate_status_update,
};

void sim_hw_init(SimHwCtx *ctx, MotorModel *model)
{
    if (ctx == NULL) { return; }
    ctx->model            = model;
    ctx->sim_dt           = 1.0f / 20000.0f;
    ctx->fast_fault_flags = 0u;
    ctx->set_output_calls = 0u;
    ctx->status.fault_code    = 0u;
    ctx->status.status        = 0u;
    ctx->status.temperature   = 25.0f;
    ctx->status.vbus          = 24.0f;
    ctx->last_vv.alpha_v      = 0.0f;
    ctx->last_vv.beta_v       = 0.0f;
}
