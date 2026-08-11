/*
 * cal_abi_align.c —— ABZ 相位校准实现（方向检测 + 相位对齐）
 *
 * 流程：拉起点 → 正转一圈判定方向 → 回固定 d 轴矢量对齐 → 反推 encoder_zero → 去磁复位。
 *
 * 对齐必须用 Vd（vq=0）：逆 Park 后电压矢量角 = θ，转子磁链轴跟到 θ。
 * 若误用 Vq，矢量在 θ+π/2，encoder_zero 系统性偏 90° → 闭环 d/q 错轴、乱转过流。
 */
#include "control/cal_abi_align.h"
#include "foc_types.h"
#include <math.h>

#define CAL_ABI_TWO_PI      (6.283185307179586f)
#define CAL_ABI_WAIT_RESET_MS (10u)   /* 复位电压后等待 */

int cal_abi_align_phase(void *ctx)
{
    CalAbiAlignCtx *c = (CalAbiAlignCtx *)ctx;
    float vd;
    float vq;
    float theta;
    float mech_start;
    float mech_end;
    float mech_aligned;
    float mech_eff;
    uint32_t i;

    if ((c == NULL) || (c->set_elec_voltage == NULL) ||
        (c->get_mech_angle == NULL) || (c->wait_ms == NULL)) {
        return FOC_ERROR;
    }
    if ((c->pole_pairs == 0u) || (c->align_voltage <= 0.0f)) {
        return FOC_ERROR;
    }

    /* 缺省值 */
    if (c->scan_steps == 0u)     { c->scan_steps = 500u; }
    if (c->scan_step_ms == 0u)   { c->scan_step_ms = 5u; }
    if (c->align_dwell_ms == 0u) { c->align_dwell_ms = 700u; }
    if (c->min_move_rad <= 0.0f) { c->min_move_rad = 0.05f; }
    if (c->pp_tol <= 0.0f)       { c->pp_tol = 0.05f; }

    vd = c->align_voltage;
    vq = 0.0f;   /* d 轴对齐（ODrive/SimpleFOC/VESC）：电压矢量角 = θ，转子 d 轴跟到 θ。
                    切勿用 Vq：逆 Park 后矢量在 θ+π/2，zero 会偏 90° → 闭环乱转/过流烧线。 */

    /* ① 拉到对齐起点，建立基准 */
    (void)c->set_elec_voltage(c->hw, c->align_theta_rad, vd, vq);
    (void)c->wait_ms(c->hw, c->align_dwell_ms);
    if (c->get_mech_angle(c->hw, &mech_start) != FOC_OK) { return FOC_ERROR; }

    /* ② 方向检测：正向旋转一圈（电角度增大），比较机械角增减 */
    for (i = 0u; i < c->scan_steps; i++) {
        theta = c->align_theta_rad + ((float)i * CAL_ABI_TWO_PI / (float)c->scan_steps);
        (void)c->set_elec_voltage(c->hw, theta, vd, vq);
        (void)c->wait_ms(c->hw, c->scan_step_ms);
    }
    if (c->get_mech_angle(c->hw, &mech_end) != FOC_OK) { return FOC_ERROR; }

    /* 移动量过小 → 转子未跟随（电压太小 / 卡死 / 编码器异常） */
    if (fabsf(mech_end - mech_start) <= c->min_move_rad) { return FOC_ERROR; }

    /* 电角度增时机械角增 → 方向一致；减 → 编码器方向反向 */
    c->inverted = (mech_end < mech_start);

    /* 响应/极对数校验（对齐 SimpleFOC pp_check + ODrive calib_scan_response）：
       正向旋转一圈 = 2π 电角 → 机械角应移动 2π/pp；|moved·pp − 2π| 超相对容差
       → 极对数配错 / 转子未跟满一圈（电压不足/堵转），阻止错误对齐产生错误 zero */
    {
        float moved = fabsf(mech_end - mech_start);
        c->pp_ok = (fabsf(moved * (float)c->pole_pairs - CAL_ABI_TWO_PI)
                    <= c->pp_tol * CAL_ABI_TWO_PI);
        if (!c->pp_ok) { return FOC_ERROR; }
    }

    /* ③ 相位对齐：回到固定电角度矢量，等转子对齐，读机械角 */
    (void)c->set_elec_voltage(c->hw, c->align_theta_rad, vd, vq);
    (void)c->wait_ms(c->hw, c->align_dwell_ms);
    if (c->get_mech_angle(c->hw, &mech_aligned) != FOC_OK) { return FOC_ERROR; }

    /* ④ 反推 encoder_zero：θ_align = mech_eff·pp + zero（mech_eff 含 inverted 校正） */
    mech_eff = c->inverted ? -mech_aligned : mech_aligned;
    c->encoder_zero = c->align_theta_rad - (mech_eff * (float)c->pole_pairs);

    /* ⑤ 去磁复位 */
    (void)c->set_elec_voltage(c->hw, c->align_theta_rad, 0.0f, 0.0f);
    (void)c->wait_ms(c->hw, CAL_ABI_WAIT_RESET_MS);
    return FOC_OK;
}
