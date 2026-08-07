/*
 * config.c —— 参数体系基础运算（纯算法）
 */
#include "foc/config.h"

float foc_limit(float x, const Limit *lim)
{
    float lo;
    float hi;

    if (lim == NULL) {
        return x;
    }
    /* 保证 min<=max（防御：配置错误不产生反向限幅） */
    lo = (lim->min < lim->max) ? lim->min : lim->max;
    hi = (lim->max > lim->min) ? lim->max : lim->min;

    return foc_clampf(x, lo, hi);
}
