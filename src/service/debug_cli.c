/*
 * debug_cli.c —— 调试 CLI Service 实现
 *
 * 命令集（对照 SimpleFOC Commander / VESC terminal / ODrive ASCII）：
 *   help                    命令 + 参数列表
 *   status                  状态机 / 时间戳 / loop stats / 安全状态
 *   dump                    传感器 + 状态 + 参数全量快照
 *   get [name]              读参数（无 name → 全部）
 *   set <name> <value>      写参数（经 ConfigSnapshot）
 *   cmd <enable|stop|recover|calibrate|prepare|self_test>   生命周期操作
 *   cmd mode <t|v|p> <target>                               下发运动命令
 *   stream <hz>|off         周期性 CSV 遥测流（波形检查）
 *   faults                  fault 寄存器详情
 *
 * 非实时路径；只经 MotorRuntime 公开接口；输出经 print 回调（总线无关）。
 */
#include "service/debug_cli.h"
#include "foc_types.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---------------- 名称映射 ---------------- */

static const char *cli_state_name(FocState s)
{
    switch (s) {
    case FOC_STATE_CREATED:     return "CREATED";
    case FOC_STATE_INIT:        return "INIT";
    case FOC_STATE_SELF_TEST:   return "SELF_TEST";
    case FOC_STATE_CALIBRATION: return "CALIBRATION";
    case FOC_STATE_READY:       return "READY";
    case FOC_STATE_RUNNING:     return "RUNNING";
    case FOC_STATE_STOPPING:    return "STOPPING";
    case FOC_STATE_FAULT:       return "FAULT";
    case FOC_STATE_LOCK:        return "LOCK";
    default:                    return "?";
    }
}

static const char *cli_mode_name(ControlMode m)
{
    switch (m) {
    case CTRL_MODE_TORQUE:   return "TORQUE";
    case CTRL_MODE_VELOCITY: return "VELOCITY";
    case CTRL_MODE_POSITION: return "POSITION";
    default:                 return "?";
    }
}

static const char *cli_fault_name(FaultCode c)
{
    switch (c) {
    case FAULT_OVERCURRENT:     return "OVERCURRENT";
    case FAULT_OVERVOLTAGE:     return "OVERVOLTAGE";
    case FAULT_UNDERVOLTAGE:    return "UNDERVOLTAGE";
    case FAULT_OVERTEMP:        return "OVERTEMP";
    case FAULT_OVERTEMP_HARD:   return "OVERTEMP_HARD";
    case FAULT_OVERVELOCITY:    return "OVERVELOCITY";
    case FAULT_ENCODER_LOSS:    return "ENCODER_LOSS";
    case FAULT_ENCODER_QUALITY: return "ENCODER_QUALITY";
    case FAULT_TIMING:          return "TIMING";
    case FAULT_CALIBRATION:     return "CALIBRATION";
    case FAULT_SELF_TEST:       return "SELF_TEST";
    case FAULT_GATE_DRIVER:     return "GATE_DRIVER";
    case FAULT_COMM_TIMEOUT:    return "COMM_TIMEOUT";
    case FAULT_CONTROL_OVERRUN: return "CONTROL_OVERRUN";
    case FAULT_SAFETY:          return "SAFETY";
    default:                    return "?";
    }
}

/* ---------------- 输出辅助 ---------------- */

static void cli_puts(DebugCli *c, const char *s)
{
    if (c->print != NULL) { c->print(c->out_hw, s); }
}

static void cli_printf(DebugCli *c, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(c->fmt_buf, sizeof(c->fmt_buf), fmt, ap);
    va_end(ap);
    cli_puts(c, c->fmt_buf);
}

/* ---------------- 解析辅助（无 strtok：MISRA 友好） ---------------- */

static const char *cli_skip_sp(const char *s)
{
    while ((*s == ' ') || (*s == '\t') || (*s == '\r') || (*s == '\n')) { s++; }
    return s;
}

/* 取下一个 token（跳过前导空白）；返回起始指针，长度写 *len */
static const char *cli_token(const char *s, uint32_t *len)
{
    const char *start;
    s = cli_skip_sp(s);
    start = s;
    while ((*s != '\0') && (*s != ' ') && (*s != '\t') && (*s != '\r') && (*s != '\n')) { s++; }
    *len = (uint32_t)(s - start);
    return start;
}

static bool cli_match(const char *tok, uint32_t len, const char *name)
{
    return ((strlen(name) == (size_t)len) && (strncmp(tok, name, len) == 0));
}

/* ---------------- 参数表（MotorRuntimeConfig 运行期可调，经 ConfigSnapshot） ---------------- */

typedef struct {
    const char *name;
    const char *help;
    float (*get)(const MotorRuntimeConfig *cfg);
    void (*set)(MotorRuntimeConfig *cfg, float v);
} CliParam;

static float p_limit_torque_max(const MotorRuntimeConfig *c) { return c->limits.limit[CTRL_MODE_TORQUE].max; }
static float p_limit_torque_min(const MotorRuntimeConfig *c) { return c->limits.limit[CTRL_MODE_TORQUE].min; }
static float p_limit_vel_max(const MotorRuntimeConfig *c)    { return c->limits.limit[CTRL_MODE_VELOCITY].max; }
static float p_limit_vel_min(const MotorRuntimeConfig *c)    { return c->limits.limit[CTRL_MODE_VELOCITY].min; }
static float p_limit_pos_max(const MotorRuntimeConfig *c)    { return c->limits.limit[CTRL_MODE_POSITION].max; }
static float p_limit_pos_min(const MotorRuntimeConfig *c)    { return c->limits.limit[CTRL_MODE_POSITION].min; }
static float p_kp(const MotorRuntimeConfig *c)               { return c->kp; }
static float p_ki(const MotorRuntimeConfig *c)               { return c->ki; }
static float p_kd(const MotorRuntimeConfig *c)               { return c->kd; }

static void s_limit_torque_max(MotorRuntimeConfig *c, float v) { c->limits.limit[CTRL_MODE_TORQUE].max = v; }
static void s_limit_torque_min(MotorRuntimeConfig *c, float v) { c->limits.limit[CTRL_MODE_TORQUE].min = v; }
static void s_limit_vel_max(MotorRuntimeConfig *c, float v)    { c->limits.limit[CTRL_MODE_VELOCITY].max = v; }
static void s_limit_vel_min(MotorRuntimeConfig *c, float v)    { c->limits.limit[CTRL_MODE_VELOCITY].min = v; }
static void s_limit_pos_max(MotorRuntimeConfig *c, float v)    { c->limits.limit[CTRL_MODE_POSITION].max = v; }
static void s_limit_pos_min(MotorRuntimeConfig *c, float v)    { c->limits.limit[CTRL_MODE_POSITION].min = v; }
static void s_kp(MotorRuntimeConfig *c, float v)               { c->kp = v; }
static void s_ki(MotorRuntimeConfig *c, float v)               { c->ki = v; }
static void s_kd(MotorRuntimeConfig *c, float v)               { c->kd = v; }

static const CliParam cli_params[] = {
    { "limit.torque.max",    "转矩上限 [V]", p_limit_torque_max, s_limit_torque_max },
    { "limit.torque.min",    "转矩下限 [V]", p_limit_torque_min, s_limit_torque_min },
    { "limit.velocity.max",  "速度上限 [rad/s]", p_limit_vel_max, s_limit_vel_max },
    { "limit.velocity.min",  "速度下限 [rad/s]", p_limit_vel_min, s_limit_vel_min },
    { "limit.position.max",  "位置上限 [rad]", p_limit_pos_max, s_limit_pos_max },
    { "limit.position.min",  "位置下限 [rad]", p_limit_pos_min, s_limit_pos_min },
    { "kp", "位置/速度环比例增益", p_kp, s_kp },
    { "ki", "速度环积分增益", p_ki, s_ki },
    { "kd", "微分增益（V0.1 预留）", p_kd, s_kd },
};
#define CLI_PARAM_COUNT ((uint32_t)(sizeof(cli_params) / sizeof(cli_params[0])))

static const CliParam *cli_find_param(const char *tok, uint32_t len)
{
    uint32_t i;
    for (i = 0u; i < CLI_PARAM_COUNT; i++) {
        if (cli_match(tok, len, cli_params[i].name)) { return &cli_params[i]; }
    }
    return NULL;
}

/* ---------------- 命令实现 ---------------- */

static int cli_cmd_help(DebugCli *c, const char *arg)
{
    uint32_t i;
    (void)arg;
    cli_puts(c, "commands:\n"
                "  help                      this list\n"
                "  status                    state/timestamp/stats/safety\n"
                "  dump                      sensor+status+params snapshot\n"
                "  get [name]                read runtime param (all if no name)\n"
                "  set <name> <value>        write runtime param (via ConfigSnapshot)\n"
                "  cmd <enable|stop|recover|calibrate|prepare|self_test>\n"
                "  cmd mode <t|v|p> <target> send motion command\n"
                "  stream <hz>|off           periodic CSV telemetry (waveform)\n"
                "  faults                    fault register detail\n"
                "params:\n");
    for (i = 0u; i < CLI_PARAM_COUNT; i++) {
        cli_printf(c, "  %-20s %s\n", cli_params[i].name, cli_params[i].help);
    }
    return 0;
}

static int cli_cmd_status(DebugCli *c, const char *arg)
{
    MotorRuntime *rt = c->rt;
    (void)arg;
    cli_printf(c, "state=%s mode=%s ts=%llu\n",
               cli_state_name(rt->state), cli_mode_name(command_buffer_get_mode(&rt->cmd_buf)),
               (unsigned long long)rt->timestamp);
    cli_printf(c, "loop=%u max_exec_us=%u overrun=%u min_dt=%u max_dt=%u\n",
               rt->stats.loop_count, rt->stats.max_exec_us, rt->stats.overrun_count,
               rt->stats.min_dt_us, rt->stats.max_dt_us);
    if (rt->fault != NULL) {
        cli_printf(c, "fault_cur=%08lx fault_lat=%08lx\n",
                   (unsigned long)rt->fault->current, (unsigned long)rt->fault->latched);
    }
    if (rt->tel != NULL) {
        cli_printf(c, "temp=%.1f vbus=%.1f\n", rt->tel->temperature_c, rt->tel->vbus_v);
    }
    return 0;
}

static int cli_cmd_faults(DebugCli *c, const char *arg)
{
    FaultCode code;
    (void)arg;
    if (c->rt->fault == NULL) { cli_puts(c, "fault: <none>\n"); return 0; }
    cli_puts(c, "fault:\n");
    for (code = (FaultCode)1; code < FAULT_CODE_COUNT; code = (FaultCode)((int)code + 1)) {
        uint32_t bit = (uint32_t)1u << (uint32_t)code;
        if (((c->rt->fault->latched & bit) != 0u) || ((c->rt->fault->current & bit) != 0u)) {
            cli_printf(c, "  %-16s %s\n", cli_fault_name(code),
                       ((c->rt->fault->latched & bit) != 0u) ? "LATCHED" : "current");
        }
    }
    return 0;
}

static int cli_cmd_dump(DebugCli *c, const char *arg)
{
    MotorRuntime *rt = c->rt;
    FastFeedback fb;
    MotorRuntimeConfig cfg;
    uint32_t i;
    (void)arg;

    cli_puts(c, "== sensor ==\n");
    if (feedback_buffer_read(&rt->fb_buf, &fb)) {
        cli_printf(c, "angle=%.6f vel=%.4f elec=%.4f q=%d\n",
                   (double)fb.mech_angle_rad, (double)fb.mech_vel_radps, (double)fb.elec_angle_rad, (int)fb.quality);
    } else {
        cli_puts(c, "angle=<no fb>\n");
    }

    cli_puts(c, "== status ==\n");
    cli_printf(c, "state=%s mode=%s ts=%llu\n",
               cli_state_name(rt->state), cli_mode_name(command_buffer_get_mode(&rt->cmd_buf)),
               (unsigned long long)rt->timestamp);
    cli_printf(c, "loop=%u overrun=%u max_exec_us=%u\n",
               rt->stats.loop_count, rt->stats.overrun_count, rt->stats.max_exec_us);
    if (rt->tel != NULL) {
        cli_printf(c, "temp=%.1f vbus=%.1f\n", (double)rt->tel->temperature_c, (double)rt->tel->vbus_v);
    }

    cli_puts(c, "== params ==\n");
    (void)config_snapshot_read(&rt->cfg_snapshot, &cfg);
    for (i = 0u; i < CLI_PARAM_COUNT; i++) {
        cli_printf(c, "  %-20s %g\n", cli_params[i].name, (double)cli_params[i].get(&cfg));
    }
    return 0;
}

static int cli_cmd_get(DebugCli *c, const char *arg)
{
    MotorRuntimeConfig cfg;
    uint32_t n;
    const char *tok = cli_token(arg, &n);
    const CliParam *p;
    uint32_t i;

    (void)config_snapshot_read(&c->rt->cfg_snapshot, &cfg);
    if (n == 0u) {                                   /* 无参数 → 全部 */
        for (i = 0u; i < CLI_PARAM_COUNT; i++) {
            cli_printf(c, "%s = %g\n", cli_params[i].name, (double)cli_params[i].get(&cfg));
        }
        return 0;
    }
    p = cli_find_param(tok, n);
    if (p == NULL) {
        cli_printf(c, "unknown param: '%.*s'\n", (int)n, tok);
        return 1;
    }
    cli_printf(c, "%s = %g\n", p->name, (double)p->get(&cfg));
    return 0;
}

static int cli_cmd_set(DebugCli *c, const char *arg)
{
    MotorRuntimeConfig cfg;
    MotorRuntimeConfig cur;
    uint32_t n;
    const char *tok = cli_token(arg, &n);
    const CliParam *p;
    char *endp;
    float val;

    if (n == 0u) { cli_puts(c, "usage: set <name> <value>\n"); return 1; }
    p = cli_find_param(tok, n);
    if (p == NULL) { cli_printf(c, "unknown param: '%.*s'\n", (int)n, tok); return 1; }

    tok = cli_skip_sp(tok + n);
    val = strtof(tok, &endp);
    if (tok == endp) { cli_puts(c, "bad value\n"); return 1; }

    /* 读当前快照 → 改一项 → 写回快照（Service 写，Slow/Fast 每周期读一致快照） */
    (void)config_snapshot_read(&c->rt->cfg_snapshot, &cfg);
    cur = cfg;
    p->set(&cfg, val);
    config_snapshot_update(&c->rt->cfg_snapshot, &cfg);
    cli_printf(c, "%s = %g (was %g)\n", p->name, (double)val, (double)p->get(&cur));
    return 0;
}

static int cli_cmd_lifecycle(DebugCli *c, const char *action, uint32_t n)
{
    int rc;
    if (cli_match(action, n, "enable"))      { rc = motor_enable(c->rt); }
    else if (cli_match(action, n, "stop"))   { rc = motor_stop(c->rt); }
    else if (cli_match(action, n, "recover")){ rc = motor_recover(c->rt); }
    else if (cli_match(action, n, "calibrate")){ rc = motor_calibrate(c->rt); }
    else if (cli_match(action, n, "prepare")){ rc = motor_prepare(c->rt); }
    else if (cli_match(action, n, "self_test")){ rc = motor_self_test(c->rt); }
    else {
        cli_puts(c, "usage: cmd <enable|stop|recover|calibrate|prepare|self_test>\n");
        return 1;
    }
    cli_printf(c, "state=%s rc=%d\n", cli_state_name(c->rt->state), rc);
    return (rc == FOC_OK) ? 0 : 1;
}

static int cli_cmd_mode(DebugCli *c, const char *arg)
{
    MotorCommand cmd;
    ControlMode mode;
    uint32_t n;
    const char *tok = cli_token(arg, &n);
    char *endp;
    float target;

    if (cli_match(tok, n, "t"))      { mode = CTRL_MODE_TORQUE; }
    else if (cli_match(tok, n, "v")) { mode = CTRL_MODE_VELOCITY; }
    else if (cli_match(tok, n, "p")) { mode = CTRL_MODE_POSITION; }
    else { cli_puts(c, "usage: cmd mode <t|v|p> <target>\n"); return 1; }

    tok = cli_skip_sp(tok + n);
    target = strtof(tok, &endp);
    if (tok == endp) { cli_puts(c, "bad target\n"); return 1; }

    cmd.target = target;
    cmd.mode   = mode;
    cmd.sequence = c->rt->cmd_buf.last_index + 1u;   /* 递增序列（Service 侧估算；Fast 以 buffer 为准） */
    (void)command_buffer_write(&c->rt->cmd_buf, &cmd);
    cli_printf(c, "cmd mode=%s target=%g\n", cli_mode_name(mode), (double)target);
    return 0;
}

static int cli_cmd_cmd(DebugCli *c, const char *arg)
{
    uint32_t n;
    const char *tok = cli_token(arg, &n);
    if (n == 0u) { cli_puts(c, "usage: cmd <action|mode> ...\n"); return 1; }
    if (cli_match(tok, n, "mode")) { return cli_cmd_mode(c, arg + n); }
    return cli_cmd_lifecycle(c, tok, n);
}

static int cli_cmd_stream(DebugCli *c, const char *arg)
{
    uint32_t n;
    const char *tok = cli_token(arg, &n);
    char *endp;
    unsigned long hz;

    if ((n == 0u) || cli_match(tok, n, "off") || cli_match(tok, n, "0")) {
        debug_cli_stream_stop(c);
        cli_puts(c, "stream off\n");
        return 0;
    }
    hz = strtoul(tok, &endp, 10);
    if (tok == endp) { cli_puts(c, "usage: stream <hz>|off\n"); return 1; }
    if (hz > 1000u) { hz = 1000u; }
    c->stream_hz = (uint8_t)hz;
    c->stream_div = 0u;
    c->stream_seq = 0u;
    cli_puts(c, "# t_us,angle_rad,vel_radps,elec_rad,state,temp_c,vbus_v,fault_lat\n");
    cli_printf(c, "stream %lu Hz\n", hz);
    return 0;
}

/* ---------------- 入口 ---------------- */

static int cli_dispatch(DebugCli *c, const char *line)
{
    uint32_t n;
    const char *tok = cli_token(line, &n);
    const char *rest;

    if (n == 0u) { return 0; }                       /* 空行 */
    rest = cli_skip_sp(line + n);

    if (cli_match(tok, n, "help"))   { return cli_cmd_help(c, rest); }
    if (cli_match(tok, n, "status")) { return cli_cmd_status(c, rest); }
    if (cli_match(tok, n, "dump"))   { return cli_cmd_dump(c, rest); }
    if (cli_match(tok, n, "get"))    { return cli_cmd_get(c, rest); }
    if (cli_match(tok, n, "set"))    { return cli_cmd_set(c, rest); }
    if (cli_match(tok, n, "cmd"))    { return cli_cmd_cmd(c, rest); }
    if (cli_match(tok, n, "stream")) { return cli_cmd_stream(c, rest); }
    if (cli_match(tok, n, "faults")) { return cli_cmd_faults(c, rest); }

    cli_printf(c, "unknown cmd: '%.*s' (help)\n", (int)n, tok);
    return 1;
}

void debug_cli_init(DebugCli *c, MotorRuntime *rt, void (*print)(void *hw, const char *s), void *out_hw)
{
    c->rt = rt;
    c->print = print;
    c->out_hw = out_hw;
    c->line_len = 0u;
    c->stream_hz = 0u;
    c->stream_div = 0u;
    c->stream_seq = 0u;
    c->line_buf[0] = '\0';
}

int debug_cli_exec(DebugCli *c, const char *line)
{
    if ((c == NULL) || (line == NULL)) { return 1; }
    return cli_dispatch(c, line);
}

int debug_cli_putc(DebugCli *c, char ch)
{
    if (c == NULL) { return 1; }
    if ((ch == '\n') || (ch == '\r')) {
        if (c->line_len > 0u) {
            c->line_buf[c->line_len] = '\0';
            (void)cli_dispatch(c, c->line_buf);
            c->line_len = 0u;
        }
        return 0;
    }
    if (c->line_len < (CLI_LINE_MAX - 1u)) {
        c->line_buf[c->line_len] = ch;
        c->line_len++;
    }
    return 0;   /* 超长 → 丢弃该字符（不阻塞） */
}

void debug_cli_stream_tick(DebugCli *c)
{
    MotorRuntime *rt;
    FastFeedback fb;
    uint8_t hz;
    uint32_t div;

    if (c == NULL) { return; }
    rt = c->rt;
    hz = c->stream_hz;
    if (hz == 0u) { return; }

    div = 1000u / (uint32_t)hz;                       /* 假设 stream_tick 每 1ms 调用 */
    c->stream_div++;
    if (c->stream_div < div) { return; }
    c->stream_div = 0u;
    c->stream_seq++;

    (void)feedback_buffer_read(&rt->fb_buf, &fb);
    cli_printf(c, "%llu,%.6f,%.4f,%.4f,%u,%.1f,%.1f,%lu\n",
               (unsigned long long)rt->timestamp,
               (double)fb.mech_angle_rad, (double)fb.mech_vel_radps, (double)fb.elec_angle_rad,
               (unsigned)rt->state,
               (rt->tel != NULL) ? (double)rt->tel->temperature_c : 0.0,
               (rt->tel != NULL) ? (double)rt->tel->vbus_v : 0.0,
               (rt->fault != NULL) ? (unsigned long)rt->fault->latched : 0ul);
}

void debug_cli_stream_stop(DebugCli *c)
{
    if (c == NULL) { return; }
    c->stream_hz = 0u;
    c->stream_div = 0u;
    c->stream_seq = 0u;
}
