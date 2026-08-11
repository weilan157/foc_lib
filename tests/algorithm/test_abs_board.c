/*
 * test_abs_board.c —— 绝对编码器相位校准端到端（仿真 abs_board 打通）
 *
 * 用 abs_board（理想电机 + 仿真绝对编码器，总线无关）把 enc_abs + cal_abi_align
 * 串成完整链路：绝对编码器无需找零，直接"方向检测 + 相位对齐"得 inverted/encoder_zero，
 * 回填后验证 foc_calc_elec_angle 与施加电角度一致。
 */
#include "control/cal_abi_align.h"
#include "device/encoder/enc_abs.h"
#include "device/encoder/encoder.h"
#include "simulation/abs_board.h"
#include "foc/foc_math.h"   /* foc_calc_elec_angle */
#include "../test_assert.h"

/* ---- 测试夹具：把 abs_board 接给 enc_abs + cal_abi_align ---- */
typedef struct {
    AbsBoard          board;
    EncAbsCtx         enc;
    EncAbsHwOps       enc_hw;
    CalAbiAlignCtx    align;
    uint32_t          pole_pairs;
    float             encoder_zero;   /* 校准结果回填 */
    bool              inverted;
} AbsRig;

static int rig_set_voltage(void *hw, float th, float vd, float vq)
{
    AbsRig *r = (AbsRig *)hw;
    int rc = abs_board_set_elec_voltage(&r->board, th, vd, vq);
    /* 模拟控制回路：施加电压后编码器实时 update，位置反馈随之更新 */
    if (rc == 0) { (void)enc_abs_update(&r->enc); }
    return rc;
}
static int rig_get_mech(void *hw, float *mech)
{
    AbsRig *r = (AbsRig *)hw;
    /* 校准读"连续机械角"：board 层负责把绝对编码器反馈连续化（回绕/方向） */
    return abs_board_get_mech_angle(&r->board, mech);
}
static int rig_wait(void *hw, uint32_t ms)
{
    return abs_board_wait_ms(&((AbsRig *)hw)->board, ms);
}

static void rig_init(AbsRig *r, uint32_t pp, float bias, bool inv, uint32_t res)
{
    abs_board_init(&r->board, pp, bias, inv, res);
    r->pole_pairs = pp;

    /* enc_abs（总线无关绝对编码器） */
    r->enc_hw.read_abs     = abs_board_read_abs;
    r->enc_hw.read_multi   = NULL;
    r->enc_hw.get_tick_us  = abs_board_get_tick_us;
    r->enc_hw.hw           = &r->board;
    r->enc.hw_ops          = &r->enc_hw;
    r->enc.resolution      = res;
    r->enc.data_err_limit  = 3u;
    r->enc.inverted        = false;   /* 校准前：方向未知 */
    CHECK(enc_abs_init(&r->enc) == 0);

    /* 相位校准（CalibrationOps.phase；绝对编码器无需 encoder 找零） */
    r->align.set_elec_voltage = rig_set_voltage;
    r->align.get_mech_angle   = rig_get_mech;
    r->align.wait_ms          = rig_wait;
    r->align.hw               = r;
    r->align.pole_pairs       = pp;
    r->align.align_voltage    = 1.5f;
}

int main(void)
{
    /* 场景：pp=4，编码器零偏 0.2 rad，方向反（模拟硬件接反），分辨率 16384 */
    AbsRig rig = {0};
    EncoderFeedback fb;
    float X;
    float elec;

    rig_init(&rig, 4u, 0.2f, true, 16384u);

    /* ① 相位校准：方向 + 对齐 + 响应/极对数校验（绝对编码器上电即绝对角，无需找零） */
    CHECK(cal_abi_align_phase(&rig.align) == 0);
    CHECK(rig.align.inverted == true);              /* 编码器方向反 */
    CHECK(rig.align.pp_ok == true);                 /* 响应/极对数校验通过 */
    /* -bias·pp = -0.8；容差 = ±1 计数量化极限（2π/res·pp ≈ 0.0015 rad 电角） */
    CHECK_NEAR(rig.align.encoder_zero, -0.2f * 4.0f, 0.02f);

    /* ② 回填（board 职责）：enc.inverted ← 校准结果；encoder_zero 保存 */
    rig.enc.inverted = rig.align.inverted;
    rig.inverted     = rig.align.inverted;
    rig.encoder_zero = rig.align.encoder_zero;

    /* ③ 端到端验证：施加已知电角度 X，读 enc_abs 反馈 mech，算电角度应 ≈ X */
    X = 1.3f;
    (void)abs_board_set_elec_voltage(&rig.board, X, 0.0f, 1.5f);
    (void)enc_abs_update(&rig.enc);
    CHECK(enc_abs_get_feedback(&rig.enc, &fb) == 0);
    elec = foc_calc_elec_angle(fb.mech_angle_rad, rig.pole_pairs, rig.encoder_zero);
    CHECK_NEAR(elec, X, 0.05f);   /* 允许 ±一个计数量化误差 */

    /* ④ 反方向位置也正确 */
    X = -0.7f;
    (void)abs_board_set_elec_voltage(&rig.board, X, 0.0f, 1.5f);
    (void)enc_abs_update(&rig.enc);
    CHECK(enc_abs_get_feedback(&rig.enc, &fb) == 0);
    elec = foc_calc_elec_angle(fb.mech_angle_rad, rig.pole_pairs, rig.encoder_zero);
    CHECK_NEAR(elec, X, 0.05f);

    TEST_REPORT();
}
