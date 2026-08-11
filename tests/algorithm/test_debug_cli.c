/*
 * test_debug_cli.c —— 调试 CLI Service 单测
 *
 * 覆盖：help/status/dump/get/set/cmd/stream/faults、
 *       参数读写经 ConfigSnapshot 生效、运动命令经 CommandBuffer、
 *       逐字符输入（putc）与直接执行（exec）、遥测流 CSV。
 */
#include "service/debug_cli.h"
#include "control/controller.h"
#include "foc/config.h"
#include "runtime/command.h"
#include "runtime/config_snapshot.h"
#include "runtime/feedback.h"
#include "runtime/stats.h"
#include "runtime/telemetry.h"
#include "safety/fault.h"
#include "foc_types.h"
#include "../test_assert.h"
#include <string.h>

/* ---- 输出捕获 ---- */
static char   g_out[16384];
static uint32_t g_out_len;

static void capture(void *hw, const char *s)
{
    (void)hw;
    while ((*s != '\0') && (g_out_len < (sizeof(g_out) - 1u))) {
        g_out[g_out_len++] = *s;
        s++;
    }
    g_out[g_out_len] = '\0';
}
static void capture_reset(void) { g_out[0] = '\0'; g_out_len = 0u; }
static bool out_contains(const char *sub) { return (strstr(g_out, sub) != NULL); }

/* ---- fake controller ops（motor_enable/stop 需要非空） ---- */
static int fake_on_enter(void *ctx, ControlMode m) { (void)ctx; (void)m; return 0; }
static int fake_on_exit(void *ctx, ControlMode m)  { (void)ctx; (void)m; return 0; }
static int fake_reset(void *ctx)                   { (void)ctx; return 0; }
static const ControllerOps fake_ops = {
    .on_enter = fake_on_enter,
    .on_exit  = fake_on_exit,
    .reset    = fake_reset,
};

static MotorRuntime g_rt;

static void rig_build(void)
{
    static MotorRuntimeConfig cfg;
    static FaultReg  fault;
    static Telemetry tel;

    memset(&g_rt, 0, sizeof(g_rt));
    memset(&cfg, 0, sizeof(cfg));
    cfg.kp = 1.0f;
    cfg.ki = 0.5f;
    cfg.kd = 0.0f;
    cfg.limits.limit[CTRL_MODE_VELOCITY].max = 10.0f;
    cfg.limits.limit[CTRL_MODE_VELOCITY].min = -10.0f;
    cfg.limits.limit[CTRL_MODE_TORQUE].max = 12.0f;
    cfg.limits.limit[CTRL_MODE_TORQUE].min = -12.0f;
    cfg.limits.limit[CTRL_MODE_POSITION].max = 3.14f;
    cfg.limits.limit[CTRL_MODE_POSITION].min = -3.14f;

    g_rt.state = FOC_STATE_READY;
    g_rt.timestamp = 12345u;
    command_buffer_init(&g_rt.cmd_buf);
    config_snapshot_init(&g_rt.cfg_snapshot, &cfg);
    feedback_buffer_init(&g_rt.fb_buf);
    stats_init(&g_rt.stats);
    g_rt.controller.ops = &fake_ops;
    g_rt.controller.ctx = NULL;

    fault_init(&fault);
    g_rt.fault = &fault;
    telemetry_init(&tel);
    tel.temperature_c = 42.5f;
    tel.vbus_v = 24.0f;
    g_rt.tel = &tel;
}

int main(void)
{
    DebugCli cli;
    MotorRuntimeConfig cfg;
    MotorCommand cmd;
    uint32_t i;

    rig_build();
    debug_cli_init(&cli, &g_rt, capture, NULL);
    capture_reset();

    /* ---- help ---- */
    CHECK(debug_cli_exec(&cli, "help") == 0);
    CHECK(out_contains("commands"));
    CHECK(out_contains("limit.velocity.max"));
    CHECK(out_contains("kp"));
    capture_reset();

    /* ---- status ---- */
    CHECK(debug_cli_exec(&cli, "status") == 0);
    CHECK(out_contains("state=READY"));
    CHECK(out_contains("ts=12345"));
    CHECK(out_contains("temp=42.5"));
    capture_reset();

    /* ---- get：全部 + 单个 + 未知 ---- */
    CHECK(debug_cli_exec(&cli, "get") == 0);
    CHECK(out_contains("kp = 1"));
    CHECK(out_contains("limit.torque.max = 12"));
    capture_reset();
    CHECK(debug_cli_exec(&cli, "get kp") == 0);
    CHECK(out_contains("kp = 1"));
    capture_reset();
    CHECK(debug_cli_exec(&cli, "get nope") != 0);
    CHECK(out_contains("unknown param"));
    capture_reset();

    /* ---- set：经 ConfigSnapshot 生效 ---- */
    CHECK(debug_cli_exec(&cli, "set kp 2.5") == 0);
    CHECK(out_contains("kp = 2.5"));
    (void)config_snapshot_read(&g_rt.cfg_snapshot, &cfg);
    CHECK_NEAR(cfg.kp, 2.5f, 1e-4f);
    capture_reset();
    CHECK(debug_cli_exec(&cli, "set limit.torque.max 15") == 0);
    (void)config_snapshot_read(&g_rt.cfg_snapshot, &cfg);
    CHECK_NEAR(cfg.limits.limit[CTRL_MODE_TORQUE].max, 15.0f, 1e-4f);
    capture_reset();
    CHECK(debug_cli_exec(&cli, "set nope 1") != 0);          /* 未知参数 */
    CHECK(debug_cli_exec(&cli, "set kp") != 0);              /* 缺值 */
    capture_reset();

    /* ---- cmd 生命周期 ---- */
    CHECK(g_rt.state == FOC_STATE_READY);
    CHECK(debug_cli_exec(&cli, "cmd enable") == 0);
    CHECK(g_rt.state == FOC_STATE_RUNNING);
    CHECK(out_contains("state=RUNNING"));
    capture_reset();
    CHECK(debug_cli_exec(&cli, "cmd stop") == 0);
    CHECK(g_rt.state == FOC_STATE_READY);
    capture_reset();
    CHECK(debug_cli_exec(&cli, "cmd bogus") != 0);           /* 未知动作 */

    /* ---- cmd mode：运动命令经 CommandBuffer ---- */
    CHECK(debug_cli_exec(&cli, "cmd mode v 5.0") == 0);
    CHECK(out_contains("mode=VELOCITY"));
    CHECK(command_buffer_read(&g_rt.cmd_buf, &cmd));
    CHECK_NEAR(cmd.target, 5.0f, 1e-4f);
    CHECK(cmd.mode == CTRL_MODE_VELOCITY);
    capture_reset();
    CHECK(debug_cli_exec(&cli, "cmd mode p -1.5") == 0);
    CHECK(command_buffer_read(&g_rt.cmd_buf, &cmd));
    CHECK_NEAR(cmd.target, -1.5f, 1e-4f);
    CHECK(cmd.mode == CTRL_MODE_POSITION);
    CHECK(debug_cli_exec(&cli, "cmd mode x 1") != 0);        /* 未知模式 */
    capture_reset();

    /* ---- dump：传感器 + 状态 + 参数 ---- */
    {
        FastFeedback fb;
        fb.mech_angle_rad = 0.25f;
        fb.mech_vel_radps = 1.5f;
        fb.elec_angle_rad = 1.0f;
        fb.quality = FEEDBACK_OK;
        feedback_buffer_write(&g_rt.fb_buf, &fb);
    }
    CHECK(debug_cli_exec(&cli, "dump") == 0);
    CHECK(out_contains("angle=0.25"));
    CHECK(out_contains("vel=1.5"));
    CHECK(out_contains("state=READY"));
    CHECK(out_contains("kp"));
    capture_reset();

    /* ---- faults ---- */
    fault_raise(g_rt.fault, FAULT_OVERTEMP, SEV_WARN);
    CHECK(debug_cli_exec(&cli, "faults") == 0);
    CHECK(out_contains("OVERTEMP"));
    capture_reset();

    /* ---- stream：周期性 CSV 遥测流 ---- */
    CHECK(debug_cli_exec(&cli, "stream 100") == 0);
    CHECK(out_contains("stream 100 Hz"));
    CHECK(out_contains("# t_us,angle"));
    capture_reset();
    for (i = 0u; i < 1000u; i++) { debug_cli_stream_tick(&cli); }   /* 100Hz × 10ms → 100 行 */
    CHECK(out_contains("12345,0.250000"));                          /* ts=12345 angle=0.25 */
    CHECK(out_contains("1.5000"));                                   /* vel 列（%.4f） */
    CHECK(out_contains("42.5"));                                    /* temp 列 */
    capture_reset();
    debug_cli_exec(&cli, "stream off");
    capture_reset();
    debug_cli_stream_tick(&cli);
    CHECK(g_out_len == 0u);                                          /* 停流后无输出 */

    /* ---- putc：逐字符输入 ---- */
    capture_reset();
    {
        const char *line = "get kp\n";
        const char *p = line;
        while (*p != '\0') { debug_cli_putc(&cli, *p); p++; }
        CHECK(out_contains("kp = 2.5"));
    }
    capture_reset();
    debug_cli_putc(&cli, 's');
    debug_cli_putc(&cli, 't');
    debug_cli_putc(&cli, 'a');
    debug_cli_putc(&cli, 't');
    debug_cli_putc(&cli, 'u');
    debug_cli_putc(&cli, 's');
    debug_cli_putc(&cli, '\n');
    CHECK(out_contains("state=READY"));

    /* ---- 非法入参 ---- */
    CHECK(debug_cli_exec(NULL, "help") != 0);
    CHECK(debug_cli_exec(&cli, NULL) != 0);
    CHECK(debug_cli_putc(NULL, 'a') != 0);

    TEST_REPORT();
}
