/*
 * cal_abi_align.h —— ABZ 相位校准插件（CalibrationOps.phase 的一个实现）
 *
 * 综合三家做法（对齐 VESC/ODrive/SimpleFOC）：
 *   ① 方向检测：电角度小步正扫 N 步 → 回基准 → 反扫 N 步，比较机械角增减 → 判定 inverted
 *      （SimpleFOC alignSensor 式：不转满一圈——限流/负载下转子转不满一圈会使原整圈方案
 *        失败；小步只要求正反扫机械角都移动且方向相反。
 *        同时用小步移动量校验极对数：正扫电角度 N·step 对应机械角 N·step/pp）
 *   ② 相位对齐：施加固定电角度 θ_align 矢量，等转子对齐，读机械角 → 反推 encoder_zero
 *      （电角度 = mech·pp + zero → zero = θ_align − mech_eff·pp，mech_eff 含 inverted 校正）
 *
 * 依赖注入（board 提供回调；测试注入 fake）：
 *   - set_elec_voltage(hw, theta_el, vd, vq)：施加 dq 电压（对齐用 vd=align_voltage, vq=0）
 *   - get_mech_angle(hw, &mech_rad)：读当前机械角（编码器原始读数，未含 inverted）
 *   - wait_ms(hw, ms)：阻塞等待（RTOS delay / 空转；校准在非实时上下文执行）
 *
 * 契约：执行后结果写入本 ctx 的 encoder_zero / inverted 输出字段，
 *       由 board 在 motor_calibrate 返回后回填 MotorRuntime.calib.encoder_zero 与 enc_abi_ctx.inverted。
 * 注意：校准会驱动电机转动，需在确保可自由转动的工况执行；施加电压期间故障由外部 Safety 兜底。
 */
#ifndef FOC_CONTROL_CAL_ABI_ALIGN_H
#define FOC_CONTROL_CAL_ABI_ALIGN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 依赖注入（board 提供） */
    int  (*set_elec_voltage)(void *hw, float theta_el_rad, float vd, float vq);
    int  (*get_mech_angle)(void *hw, float *mech_rad);
    int  (*wait_ms)(void *hw, uint32_t ms);
    void *hw;

    /* 配置（board/上层设置；0/缺省取默认） */
    uint32_t pole_pairs;       /* 极对数，>0 */
    float    align_voltage;    /* 对齐电压 [V]（V0.1 电压模式），>0 */
    float    align_theta_rad;  /* 对齐电角度 [rad]，默认 0（§3.6 约定基准） */
    uint32_t align_dwell_ms;   /* 对齐稳定等待 [ms]，默认 700 */
    uint32_t scan_step_ms;     /* 方向检测每步等待 [ms]，默认 20 */
    uint32_t scan_steps;       /* 正/反扫各步数（不转一圈），默认 8 */
    float    scan_step_rad;    /* 每步电角度增量 [rad]，默认 0.1（≈5.7°，正扫总弧 = scan_steps·scan_step_rad） */
    float    min_move_rad;     /* 方向检测最小机械角移动量 [rad]，默认 0.02（防“没转”误判） */
    float    pp_tol;           /* 极对数校验相对容差（小步移动 vs 期望 N·step/pp），默认 0.05（±5%） */

    /* 输出（phase 校准结果） */
    float    encoder_zero;     /* [rad]：电角度 = mech·pp + encoder_zero */
    bool     inverted;         /* 编码器方向反向？ */
    bool     pp_ok;            /* 极对数校验通过？（正扫机械角移动 ≈ scan_steps·scan_step_rad/pp） */
} CalAbiAlignCtx;

/* CalibrationOps.phase 实现（ctx = CalAbiAlignCtx*） */
int cal_abi_align_phase(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_CAL_ABI_ALIGN_H */
