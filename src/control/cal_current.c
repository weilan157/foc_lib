/*
 * cal_current.c —— 电流零点校准实现
 *
 * 零电压（电机不转）下三相原始值均值 = 零点偏移。
 * 对应 SimpleFOC calibrateOffsets() / VESC dc_cal()。
 */
#include "control/cal_current.h"
#include "foc_types.h"

int cal_current_offset(void *ctx)
{
    CalCurrentCtx *c = (CalCurrentCtx *)ctx;
    uint32_t rounds;
    uint32_t settle;
    uint32_t i;
    double   sum_a = 0.0, sum_b = 0.0, sum_c = 0.0;

    if ((c == NULL) || (c->set_zero_voltage == NULL) ||
        (c->read_phase_raw == NULL) || (c->wait_ms == NULL)) {
        return FOC_ERROR;
    }

    rounds  = (c->rounds   == 0u) ? 64u : c->rounds;
    settle  = (c->settle_ms == 0u) ? 50u : c->settle_ms;

    /* 零电压 + 稳定（等采样链路就绪/母线稳定） */
    if (c->set_zero_voltage(c->hw) != FOC_OK) { return FOC_ERROR; }
    (void)c->wait_ms(c->hw, settle);

    for (i = 0u; i < rounds; i++) {
        float a, b, cc;
        if (c->read_phase_raw(c->hw, &a, &b, &cc) != FOC_OK) { return FOC_ERROR; }
        sum_a += (double)a;
        sum_b += (double)b;
        sum_c += (double)cc;
    }

    c->offset_a = (float)(sum_a / (double)rounds);
    c->offset_b = (float)(sum_b / (double)rounds);
    c->offset_c = (float)(sum_c / (double)rounds);
    return FOC_OK;
}
