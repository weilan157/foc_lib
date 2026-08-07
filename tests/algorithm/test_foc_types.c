/*
 * test_foc_types.c —— 基础物理量单测（夹取 / 有限性）
 */
#include "foc_types.h"
#include "../test_assert.h"
#include <math.h>

int main(void)
{
    /* foc_clampf */
    CHECK_NEAR(foc_clampf(5.0f, 0.0f, 10.0f), 5.0f, 0.0f);
    CHECK_NEAR(foc_clampf(-3.0f, 0.0f, 10.0f), 0.0f, 0.0f);
    CHECK_NEAR(foc_clampf(15.0f, 0.0f, 10.0f), 10.0f, 0.0f);

    /* foc_is_finite */
    CHECK(foc_is_finite(1.0f) == true);
    CHECK(foc_is_finite(0.0f) == true);
    CHECK(foc_is_finite(INFINITY) == false);
    CHECK(foc_is_finite(NAN) == false);

    TEST_REPORT();
}
