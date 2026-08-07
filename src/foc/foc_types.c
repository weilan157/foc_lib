/*
 * foc_types.c —— 基础物理量运算（纯算法）
 */
#include "foc_types.h"
#include <math.h>

float foc_clampf(float x, float lo, float hi)
{
    float y = x;
    if (y < lo) {
        y = lo;
    }
    if (y > hi) {
        y = hi;
    }
    return y;
}

bool foc_is_finite(float x)
{
    return (bool)isfinite((double)x);
}
