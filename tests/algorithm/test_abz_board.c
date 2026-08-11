/*
 * test_abz_board.c —— ABZ 三件套端到端（仿真 board 打通：找零 + 方向 + 对齐 + 电角度验证）
 *
 * 用 abz_board（理想电机 + 仿真 ABZ）把 enc_abi + cal_abi_find_index + cal_abi_align
 * 串成完整链路，验证校准后 foc_calc_elec_angle 与施加电角度一致。
 */
#include "control/cal_abi_find_index.h"
#include "control/cal_abi_align.h"
#include "device/encoder/enc_abi.h"
#include "simulation/abz_board.h"
#include "foc/foc_math.h"   /* foc_calc_elec_angle */
#include "../test_assert.h"

/* ---- 测试夹具：把 abz_board 接给 enc_abi + 两个校准插件 ---- */
typedef struct {
    AbzBoard          board;
    EncAbiCtx         enc;
    EncAbiHwOps       enc_hw;
    CalAbiFindIndexCtx find;
    CalAbiAlignCtx    align;
    uint32_t          pole_pairs;
    float             encoder_zero;   /* 校准结果回填 */
    bool              inverted;
} AbzRig;

static int rig_enc_read_count(void *hw, int32_t *count)
{
    return abz_board_read_count(hw, count);
}
static bool rig_enc_read_index(void *hw)
{
    return abz_board_read_index(hw);
}
static uint64_t rig_enc_tick(void *hw)
{
    return abz_board_get_tick_us(hw);
}
static int rig_set_voltage(void *hw, float th, float vd, float vq)
{
    AbzRig *r = (AbzRig *)hw;
    int rc = abz_board_set_elec_voltage(hw, th, vd, vq);
    /* 模拟控制回路：施加电压后编码器实时 update，位置反馈随之更新 */
    if (rc == 0) { (void)enc_abi_update(&r->enc); }
    return rc;
}
static int rig_sensor_update(void *hw)
{
    AbzRig *r = (AbzRig *)hw;
    return enc_abi_update(&r->enc);
}
static int rig_is_index_found(void *hw, bool *found)
{
    AbzRig *r = (AbzRig *)hw;
    *found = r->enc.index_found;
    return 0;
}
static int rig_get_mech(void *hw, float *mech)
{
    AbzRig *r = (AbzRig *)hw;
    EncoderFeedback fb;
    /* 控制回路使用的反馈：编码器相对 index 零位的机械角（rig_set_voltage 已 update） */
    (void)enc_abi_get_feedback(&r->enc, &fb);
    *mech = fb.mech_angle_rad;
    return 0;
}
static int rig_wait(void *hw, uint32_t ms)
{
    return abz_board_wait_ms(hw, ms);
}

static void rig_init(AbzRig *r, uint32_t pp, float bias, bool inv, uint32_t cpr)
{
    abz_board_init(&r->board, pp, bias, inv, cpr);
    r->pole_pairs = pp;

    /* enc_abi */
    r->enc_hw.read_count  = rig_enc_read_count;
    r->enc_hw.read_index  = rig_enc_read_index;
    r->enc_hw.get_tick_us = rig_enc_tick;
    r->enc_hw.hw          = &r->board;
    r->enc.hw_ops   = &r->enc_hw;
    r->enc.cpr      = cpr;
    r->enc.use_index = true;
    r->enc.inverted = false;          /* 校准前：方向未知 */
    CHECK(enc_abi_init(&r->enc) == 0);

    /* 找零驱动（CalibrationOps.encoder） */
    r->find.set_elec_voltage = rig_set_voltage;
    r->find.sensor_update    = rig_sensor_update;
    r->find.is_index_found   = rig_is_index_found;
    r->find.wait_ms          = rig_wait;
    r->find.hw               = r;
    r->find.pole_pairs       = pp;
    r->find.align_voltage    = 1.5f;

    /* 相位校准（CalibrationOps.phase） */
    r->align.set_elec_voltage = rig_set_voltage;
    r->align.get_mech_angle   = rig_get_mech;
    r->align.wait_ms          = rig_wait;
    r->align.hw               = r;
    r->align.pole_pairs       = pp;
    r->align.align_voltage    = 1.5f;
}

int main(void)
{
    /* 场景：pp=4，编码器零偏 0.2 rad，方向反（模拟硬件接反），cpr=1024 */
    AbzRig rig = {0};   /* 清零：避免未初始化字段（如 align_theta_rad）为栈垃圾导致校准失败 */
    EncoderFeedback fb;
    float X;
    float elec;

    rig_init(&rig, 4u, 0.2f, true, 1024u);

    /* ① 找零：开环旋转直到 index */
    CHECK(cal_abi_find_index(&rig.find) == 0);
    CHECK(rig.enc.index_found == true);

    /* ② 方向 + 对齐 */
    CHECK(cal_abi_align_phase(&rig.align) == 0);
    CHECK(rig.align.inverted == true);              /* 编码器方向反 */
    /* -bias·pp = -0.8；容差 = ±1 计数量化极限（2π/cpr·pp ≈ 0.025 rad 电角） */
    CHECK_NEAR(rig.align.encoder_zero, -0.2f * 4.0f, 0.03f);

    /* ③ 回填（board 职责）：enc.inverted ← 校准结果；encoder_zero 保存 */
    rig.enc.inverted = rig.align.inverted;
    rig.inverted     = rig.align.inverted;
    rig.encoder_zero = rig.align.encoder_zero;

    /* ④ 端到端验证：施加已知电角度 X，读 enc 机械角，算电角度应 ≈ X */
    X = 1.3f;
    (void)abz_board_set_elec_voltage(&rig.board, X, 0.0f, 1.5f);
    (void)enc_abi_update(&rig.enc);
    CHECK(enc_abi_get_feedback(&rig.enc, &fb) == 0);
    elec = foc_calc_elec_angle(fb.mech_angle_rad, rig.pole_pairs, rig.encoder_zero);
    CHECK_NEAR(elec, X, 0.05f);   /* 允许 ±一个计数量化误差 */

    /* ⑤ 反方向位置也正确 */
    X = -0.7f;
    (void)abz_board_set_elec_voltage(&rig.board, X, 0.0f, 1.5f);
    (void)enc_abi_update(&rig.enc);
    CHECK(enc_abi_get_feedback(&rig.enc, &fb) == 0);
    elec = foc_calc_elec_angle(fb.mech_angle_rad, rig.pole_pairs, rig.encoder_zero);
    CHECK_NEAR(elec, X, 0.05f);

    TEST_REPORT();
}
