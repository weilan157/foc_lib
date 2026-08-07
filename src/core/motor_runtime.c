/*
 * motor_runtime.c —— MotorRuntime 实现（§4.4 / §9 / §10.3，冻结）
 * - motor_slow_step：CommandBuffer → ConfigSnapshot → 输入限幅 → FeedbackBuffer → step_slow → SetpointBuffer
 * - motor_fast_step：HardwareAdapter 采样 → 质量/快速故障分级 → FastFeedback → step_fast → 输出限幅 → 逆Park → gate_set_output → 反馈写回
 * - motor_enter_safe_state：Fault 唯一安全关断出口（§16.1）
 */
#include "core/motor_runtime.h"
#include "foc/foc_math.h"
#include "runtime/limiter.h"
#include "foc_types.h"

int motor_init(MotorRuntime *rt, uint32_t idx, const MotorStaticConfig *scfg,
               const HwAdapterOps *hw_ops, void *hw_ctx,
               const ControllerOps *ctrl_ops, void *ctrl_ctx,
               const CalibrationOps *cal_ops, void *cal_ctx)
{
    if (rt == NULL || scfg == NULL || hw_ops == NULL || ctrl_ops == NULL) { return FOC_ERROR; }

    rt->idx     = idx;
    rt->scfg    = scfg;
    rt->state   = FOC_STATE_INIT;
    rt->timestamp = 0u;
    rt->fault   = NULL;   /* 由 board / 组装者注入（§10.3） */
    rt->tel     = NULL;

    /* 反馈唯一出口：初始化缓冲（sp_buf 两个槽都写干净值，避免读未初始化栈数据） */
    feedback_buffer_init(&rt->fb_buf);
    command_buffer_init(&rt->cmd_buf);
    {
        ControlSetpoint zero_sp;
        zero_sp.voltage_q = 0.0f;
        zero_sp.voltage_d = 0.0f;
        zero_sp.current_q = 0.0f;
        zero_sp.torque    = 0.0f;
        zero_sp.seq       = 0u;
        rt->sp_buf.data[0] = zero_sp;
        rt->sp_buf.data[1] = zero_sp;
        rt->sp_buf.index     = 0u;
        rt->sp_buf.last_index = 0u;
    }

    /* 硬件注入（MotorControl 只认识 HardwareAdapter） */
    rt->hw.ops = hw_ops;
    rt->hw.ctx = hw_ctx;

    /* Controller 静态绑定 */
    rt->controller.ops = ctrl_ops;
    rt->controller.ctx = ctrl_ctx;
    if (rt->controller.ops->init != NULL) { (void)rt->controller.ops->init(rt->controller.ctx); }

    /* 校准插件 */
    rt->calibration = (cal_ops != NULL) ? *cal_ops : (CalibrationOps){ 0 };
    (void)cal_ctx;

    fault_monitor_init(&rt->fault_monitor);
    stats_init(&rt->stats);
    safety_init(&rt->safety);
    /* tel 由组装者注入后自行 telemetry_init（§10.3），motor_init 不触碰 */

    /* 校准零（V0.1：未校准前 encoder_zero=0，校准后产生） */
    rt->calib.encoder_zero = 0.0f;
    rt->calib.phase_order  = 0u;
    rt->calib.current_offset[0] = 0.0f;
    rt->calib.current_offset[1] = 0.0f;
    rt->calib.current_offset[2] = 0.0f;

    return FOC_OK;
}

int motor_load_config(MotorRuntime *rt, const MotorRuntimeConfig *rcfg)
{
    if (rt == NULL || rcfg == NULL) { return FOC_ERROR; }
    config_snapshot_init(&rt->cfg_snapshot, rcfg);   /* 上电：活动槽 = 默认运行配置 */
    return FOC_OK;
}

int motor_self_test(MotorRuntime *rt)
{
    if (rt == NULL) { return FOC_ERROR; }
    if (rt->state != FOC_STATE_INIT) { return FOC_ERROR; }
    rt->state = FOC_STATE_SELF_TEST;
    /* V0.1：最小自检（HAL 自检由 board 负责）；失败交 FAULT */
    rt->state = FOC_STATE_CALIBRATION;
    return FOC_OK;
}

int motor_calibrate(MotorRuntime *rt)
{
    if (rt == NULL) { return FOC_ERROR; }
    if (rt->state != FOC_STATE_CALIBRATION) { return FOC_ERROR; }

    if (rt->calibration.encoder != NULL) { (void)rt->calibration.encoder(NULL); }
    if (rt->calibration.phase    != NULL) { (void)rt->calibration.phase(NULL); }

    rt->state = FOC_STATE_READY;
    return FOC_OK;
}

int motor_prepare(MotorRuntime *rt)
{
    if (rt == NULL) { return FOC_ERROR; }
    if (rt->state != FOC_STATE_CALIBRATION) { return FOC_ERROR; }
    rt->state = FOC_STATE_READY;
    return FOC_OK;
}

int motor_enable(MotorRuntime *rt)
{
    ControlMode mode;

    if (rt == NULL) { return FOC_ERROR; }
    if (rt->state != FOC_STATE_READY) { return FOC_ERROR; }   /* enable 必须从 READY */

    mode = command_buffer_get_mode(&rt->cmd_buf);
    if (rt->controller.ops->on_enter != NULL) { (void)rt->controller.ops->on_enter(rt->controller.ctx, mode); }

    rt->state = FOC_STATE_RUNNING;
    return FOC_OK;
}

int motor_stop(MotorRuntime *rt)
{
    if (rt == NULL) { return FOC_ERROR; }
    if (rt->state == FOC_STATE_RUNNING) {
        rt->state = FOC_STATE_STOPPING;
        if (rt->controller.ops->on_exit != NULL) { (void)rt->controller.ops->on_exit(rt->controller.ctx, command_buffer_get_mode(&rt->cmd_buf)); }
        if (rt->controller.ops->reset != NULL)   { (void)rt->controller.ops->reset(rt->controller.ctx); }
        rt->state = FOC_STATE_READY;
    }
    return FOC_OK;
}

int motor_recover(MotorRuntime *rt)
{
    if (rt == NULL) { return FOC_ERROR; }

    if (rt->state == FOC_STATE_FAULT) {
        if (rt->fault != NULL) {
            /* SHUTDOWN/RETRY → FAULT_CLEAR → READY */
            rt->fault->current = 0u;
            rt->fault->latched = 0u;
        }
        rt->state = FOC_STATE_READY;
        return FOC_OK;
    }
    return FOC_ERROR;   /* LOCK 只能断电上电（§9.1） */
}

/* Fault 唯一安全关断出口（§16.1，冻结） */
void motor_enter_safe_state(MotorRuntime *rt, FaultCode code)
{
    FaultAction action;

    if (rt == NULL) { return; }

    fault_raise(rt->fault, code, SEV_CRITICAL);
    action = fault_get_action(code);

    if (action == FAULT_ACTION_LATCH) {
        rt->state = FOC_STATE_LOCK;
    } else {
        rt->state = FOC_STATE_FAULT;
    }

    /* 安全关断（STO + PWM disable）经独立 Safety 层（§15），不依赖控制算法 */
    safety_enter_sto(&rt->safety);
}

void motor_slow_step(MotorRuntime *rt, const TimeBase *tb)
{
    TimeStep step;
    MotorCommand cmd;
    MotorRuntimeConfig cfg;
    FastFeedback fb;
    ControlSetpoint sp = { 0 };   /* 调用方零初始化：Controller 契约（seq 从 0 计数） */

    if (rt == NULL || rt->state != FOC_STATE_RUNNING) { return; }

    /* 0. TimeStep（方案 B）+ RuntimeConfig（唯一途径 ConfigSnapshot） */
    (void)timebase_get_step(tb, &step);
    (void)config_snapshot_read(&rt->cfg_snapshot, &cfg);   /* false=冷启动 → scfg 默认表兜底 */

    /* 1. 读命令（保持型：无新命令 → cmd=最近值） */
    (void)command_buffer_read(&rt->cmd_buf, &cmd);

    /* 2. 输入 Limiter（按模式限幅表 cfg.limits；cap 在 scfg 不在此处） */
    cmd.target = foc_limit(cmd.target, &cfg.limits.limit[cmd.mode]);

    /* 3. 读最近反馈快照（唯一来源 FeedbackBuffer） */
    (void)feedback_buffer_read(&rt->fb_buf, &fb);

    /* 4. Controller step_slow：Position P → Velocity PI → voltage_sp（PID 增益经快照 cfg） */
    if (rt->controller.ops->step_slow != NULL) {
        (void)rt->controller.ops->step_slow(rt->controller.ctx, &cmd, &fb, &cfg, step.dt, &sp);
    } else {
        sp.voltage_q = 0.0f;
        sp.voltage_d = 0.0f;
        sp.current_q = 0.0f;
        sp.torque    = 0.0f;
        sp.seq       = 0u;
    }

    /* 5. 写设定值缓冲（无锁，供 Fast 消费） */
    setpoint_write(&rt->sp_buf, &sp);
}

void motor_fast_step(MotorRuntime *rt, const TimeBase *tb)
{
    TimeStep step;
    MotorRuntimeConfig cfg;
    EncoderFeedback enc;
    FastFeedback fb;
    ControlSetpoint sp;
    ControlOutput out;
    VoltageVector vv;
    GateFastFault fast_fault;

    if (rt == NULL || rt->state != FOC_STATE_RUNNING) { return; }

    /* 0. TimeStep + RuntimeConfig（唯一途径 ConfigSnapshot） */
    (void)timebase_get_step(tb, &step);
    (void)config_snapshot_read(&rt->cfg_snapshot, &cfg);

    /* 1. 经 HardwareAdapter 采样（温度/母线走 SlowStatus，不在此处） */
    if (rt->hw.ops->sensor_update(rt->hw.ctx) != FOC_OK ||
        rt->hw.ops->sensor_get_feedback(rt->hw.ctx, &enc) != FOC_OK) {
        motor_enter_safe_state(rt, FAULT_ENCODER_LOSS);
        return;
    }

    /* 1a. 质量分级：ENC_QUALITY_BAD 硬错误 → 立即交 Fault（禁止置 angle=0） */
    if (enc.quality == ENC_QUALITY_BAD) {
        motor_enter_safe_state(rt, FAULT_ENCODER_QUALITY);
        return;
    }

    /* 1b. GateDriver FastFault 分级检查（20kHz 位读取） */
    if (rt->hw.ops->gate_fast_check(rt->hw.ctx, &fast_fault) != FOC_OK ||
        fast_fault.fault_flags != 0u) {
        motor_enter_safe_state(rt, FAULT_GATE_DRIVER);
        return;
    }

    /* 2. 组装 FastFeedback（控制关键量，带 FeedbackQuality） */
    fb.mech_angle_rad = enc.mech_angle_rad;
    fb.mech_vel_radps = enc.velocity;
    fb.elec_angle_rad = foc_calc_elec_angle(enc.mech_angle_rad, rt->scfg->pole_pairs,
                                            rt->calib.encoder_zero);
    fb.quality = (enc.quality == ENC_QUALITY_GOOD) ? FEEDBACK_OK : FEEDBACK_STALE;

    /* 3. 读 fast 设定值（Slow 产出，保持型） */
    (void)setpoint_read(&rt->sp_buf, &sp);

    /* 4. Controller step_fast：V0.1 电压直通（V0.2 插入电流环） */
    if (rt->controller.ops->step_fast != NULL) {
        (void)rt->controller.ops->step_fast(rt->controller.ctx, &fb, &sp, &cfg, step.dt, &out);
    } else {
        out.voltage_q = 0.0f;
        out.voltage_d = 0.0f;
        out.current_q = 0.0f;
        out.torque    = 0.0f;
    }

    /* 5. 输出 Limiter（cap 静态 + 限幅表快照，无半更新窗口） */
    limiter_apply(&out, &rt->scfg->cap, &cfg);

    /* 6. FOC Core → HardwareAdapter（SVM 在 GateDriver 内）
       注：foc_inverse_park 签名 = 4 参数返回 VoltageVector（foc_math.h） */
    vv = foc_inverse_park(fb.elec_angle_rad, 0.0f, out.voltage_d, out.voltage_q);
    (void)rt->hw.ops->gate_set_output(rt->hw.ctx, &vv);

    /* 7. 写反馈唯一出口 FeedbackBuffer + 性能统计 */
    feedback_buffer_write(&rt->fb_buf, &fb);
    stats_update(&rt->stats, tb);
    rt->timestamp = tb->timestamp_us;
}
