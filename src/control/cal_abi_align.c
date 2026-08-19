/*
 * cal_abi_align.c —— ABZ 相位校准实现（方向检测 + 相位对齐）
 *
 * 流程：基准对齐 → 电角度小步正扫 N 步 → 回基准 → 反扫 N 步判方向 → 对齐读零位 → 反推 encoder_zero → 去磁。
 *
 * 方向检测用 SimpleFOC alignSensor 式小步正反扫（不转满一圈）：
 *   24V + 电源限流下转子转不满一圈 → 原“整圈 + moved·pp≈2π”校验会失败；
 *   小步只要求正/反扫机械角都移动且方向相反，并校验移动量 ≈ 电角弧/pp（防极对数配错）。
 *
 * 对齐必须用 Vd（vq=0）：逆 Park 后电压矢量角 = θ，转子磁链轴跟到 θ。
 * 若误用 Vq，矢量在 θ+π/2，encoder_zero 系统性偏 90° → 闭环 d/q 错轴、乱转过流。
 */
#include "control/cal_abi_align.h"
#include "foc_types.h"
#include <math.h>

#define CAL_ABI_TWO_PI      (6.283185307179586f)
#define CAL_ABI_WAIT_RESET_MS (10u)   /* 复位电压后等待 */
#define CAL_ABI_PI          (3.141592653589793f)

/* 归一化到 (-π, π]（机械角跨零判断用） */
static float cal_wrap_pi(float a)
{
    while (a > CAL_ABI_PI)  { a -= CAL_ABI_TWO_PI; }
    while (a < -CAL_ABI_PI) { a += CAL_ABI_TWO_PI; }
    return a;
}

int cal_abi_align_phase(void *ctx)
{
    CalAbiAlignCtx *c = (CalAbiAlignCtx *)ctx;
    float vd;
    float vq;
    float theta_base;
    float mech_base;
    float mech_plus;
    float mech_minus;
    float mech_aligned;
    float plus_move;
    float minus_move;
    uint32_t i;

    if ((c == NULL) || (c->set_elec_voltage == NULL) ||
        (c->get_mech_angle == NULL) || (c->wait_ms == NULL)) {
        return FOC_ERROR;
    }
    if ((c->pole_pairs == 0u) || (c->align_voltage <= 0.0f)) {
        return FOC_ERROR;
    }

    /* 缺省值 */
    if (c->scan_steps == 0u)    { c->scan_steps = 8u; }        /* 正/反各步数 */
    if (c->scan_step_rad == 0.0f){ c->scan_step_rad = 0.1f; } /* 每步电角增量 [rad] */
    if (c->scan_step_ms == 0u)  { c->scan_step_ms = 20u; }
    if (c->align_dwell_ms == 0u){ c->align_dwell_ms = 700u; }
    if (c->min_move_rad <= 0.0f){ c->min_move_rad = 0.02f; }
    if (c->pp_tol <= 0.0f)      { c->pp_tol = 0.05f; }

    vd = c->align_voltage;
    vq = 0.0f;   /* d 轴对齐（ODrive/SimpleFOC/VESC）：电压矢量角 = θ，转子 d 轴跟到 θ。
                    切勿用 Vq：逆 Park 后矢量在 θ+π/2，zero 会偏 90° → 闭环乱转/过流烧线。 */
    theta_base = c->align_theta_rad;

    /* ① 基准对齐：建立机械角基准 */
    (void)c->set_elec_voltage(c->hw, theta_base, vd, vq);
    (void)c->wait_ms(c->hw, c->align_dwell_ms);
    if (c->get_mech_angle(c->hw, &mech_base) != FOC_OK) { return FOC_ERROR; }

    /* ② 方向检测：电角度小步正扫 N 步 → 回基准 → 反扫 N 步（SimpleFOC alignSensor 式） */
    for (i = 1u; i <= c->scan_steps; i++) {
        (void)c->set_elec_voltage(c->hw, theta_base + (float)i * c->scan_step_rad, vd, vq);
        (void)c->wait_ms(c->hw, c->scan_step_ms);
    }
    if (c->get_mech_angle(c->hw, &mech_plus) != FOC_OK) { return FOC_ERROR; }
    plus_move = cal_wrap_pi(mech_plus - mech_base);

    /* 回基准 */
    (void)c->set_elec_voltage(c->hw, theta_base, vd, vq);
    (void)c->wait_ms(c->hw, c->align_dwell_ms);

    for (i = 1u; i <= c->scan_steps; i++) {
        (void)c->set_elec_voltage(c->hw, theta_base - (float)i * c->scan_step_rad, vd, vq);
        (void)c->wait_ms(c->hw, c->scan_step_ms);
    }
    if (c->get_mech_angle(c->hw, &mech_minus) != FOC_OK) { return FOC_ERROR; }
    minus_move = cal_wrap_pi(mech_minus - mech_base);

    /* 移动量过小 → 转子未跟随（电压太小 / 限流 / 卡死 / 编码器异常） */
    if ((fabsf(plus_move) <= c->min_move_rad) || (fabsf(minus_move) <= c->min_move_rad)) {
        return FOC_ERROR;
    }
    /* 正反扫应反向（同号 = 转子被拖着单方向跑，异常） */
    if ((plus_move * minus_move) >= 0.0f) { return FOC_ERROR; }

    /* 电角度正向增大时机械角增 → 方向一致；减 → 编码器方向反向 */
    c->inverted = (plus_move < 0.0f);

    /* 极对数校验（小步移动量 vs 期望 N·step/pp，防极对数配错存错 zero）：
       正扫电角弧 = scan_steps·scan_step_rad → 转子应移动 scan_steps·scan_step_rad/pp */
    {
        float expected = ((float)c->scan_steps * c->scan_step_rad) / (float)c->pole_pairs;
        float actual   = fabsf(plus_move);
        c->pp_ok = (fabsf(actual - expected) <= c->pp_tol * expected);
        if (!c->pp_ok) { return FOC_ERROR; }
    }

    /* ③ 相位对齐：回到基准电角度矢量，等转子对齐，读机械角 */
    (void)c->set_elec_voltage(c->hw, theta_base, vd, vq);
    (void)c->wait_ms(c->hw, c->align_dwell_ms);
    if (c->get_mech_angle(c->hw, &mech_aligned) != FOC_OK) { return FOC_ERROR; }

    /* ④ 反推 encoder_zero：θ_align = mech_eff·pp + zero（mech_eff 含 inverted 校正） */
    {
        float mech_eff = c->inverted ? -mech_aligned : mech_aligned;
        c->encoder_zero = theta_base - (mech_eff * (float)c->pole_pairs);
    }

    /* ⑤ 去磁复位 */
    (void)c->set_elec_voltage(c->hw, theta_base, 0.0f, 0.0f);
    (void)c->wait_ms(c->hw, CAL_ABI_WAIT_RESET_MS);
    return FOC_OK;
}
