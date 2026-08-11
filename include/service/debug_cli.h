/*
 * debug_cli.h —— 调试 CLI Service（service/，§10.3 Debug 全走 Service）
 *
 * V0.1.6 实现阶段 —— 串口命令行调试（对照 SimpleFOC Commander / VESC terminal / ODrive ASCII）：
 *   · 参数配置：get/set（运行期参数经 ConfigSnapshot 写，禁直写 MotorRuntime）
 *   · 操作命令：cmd enable/stop/recover/calibrate/...（经 motor_* 生命周期 API）
 *   · 传感器数据：dump（经 FeedbackBuffer 读角度/速度/电角度）
 *   · 状态数据：status/faults（状态机/时间戳/loop stats/fault 寄存器）
 *   · 波形检查：stream <hz> 周期性 CSV 遥测流（board 在 Slow Task 周期调 stream_tick）
 *
 * 约束（冻结架构）：
 *   · 非实时路径：禁止在 fast_step 内调用（§12）
 *   · 只经公开接口：cmd_buf / cfg_snapshot / fb_buf / motor_* API / stats / fault / tel
 *   · 输出总线无关：注入 print 回调（UART/虚拟串口/CAN 文本等由 board 提供）
 *   · 单电机一实例（多电机多实例，board 装配）
 */
#ifndef FOC_SERVICE_DEBUG_CLI_H
#define FOC_SERVICE_DEBUG_CLI_H

#include <stdint.h>
#include "core/motor_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLI_LINE_MAX   96u   /* 单行命令最长字节 */
#define CLI_FMT_MAX    128u  /* 格式化缓冲 */

typedef struct {
    /* 输出回调（board 注入；可逐段调用） */
    void (*print)(void *hw, const char *s);
    void *out_hw;

    MotorRuntime *rt;            /* 绑定的电机运行时（单电机一实例） */

    /* 行输入缓冲（debug_cli_putc 逐字符填充，遇 '\n' 执行） */
    char   line_buf[CLI_LINE_MAX];
    uint8_t line_len;

    /* 遥测流（波形）：board 在 Slow Task 周期调 debug_cli_stream_tick */
    uint8_t  stream_hz;          /* 0 = 停流；>0 = 每 1s/stream_hz 输出一行 */
    uint32_t stream_div;         /* 节流计数（假设 stream_tick 1kHz 调用） */
    uint32_t stream_seq;         /* 已输出行数 */

    char fmt_buf[CLI_FMT_MAX];   /* 格式化缓冲（Service 非实时，允许 vsnprintf） */
} DebugCli;

/* 初始化（rt 必须已 motor_init + motor_load_config；print 可 NULL → 静默） */
void debug_cli_init(DebugCli *c, MotorRuntime *rt, void (*print)(void *hw, const char *s), void *out_hw);

/* 逐字符喂入（UART RX ISR / 轮询）；遇 '\n' 或 '\r' 执行一行。返回 0=正常 */
int  debug_cli_putc(DebugCli *c, char ch);

/* 直接执行一行（测试 / 批量注入）；成功返回 0，命令未知/参数错返回非 0 */
int  debug_cli_exec(DebugCli *c, const char *line);

/* 遥测流节拍（board 在 Slow Task 每 1ms 调一次；按 stream_hz 节流输出 CSV 行） */
void debug_cli_stream_tick(DebugCli *c);

/* 停止遥测流 */
void debug_cli_stream_stop(DebugCli *c);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SERVICE_DEBUG_CLI_H */
