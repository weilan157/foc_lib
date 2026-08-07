/*
 * test_assert.h —— 极简 C 单测断言（每个测试可执行文件单独使用）
 */
#ifndef TEST_ASSERT_H
#define TEST_ASSERT_H

#include <stdio.h>
#include <math.h>

static int g_test_failures = 0;
static int g_test_checks   = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_test_checks++;                                                       \
        if (!(cond)) {                                                         \
            g_test_failures++;                                                 \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                  \
    do {                                                                       \
        double _d = fabs((double)(a) - (double)(b));                           \
        g_test_checks++;                                                       \
        if (!(_d <= (double)(eps))) {                                          \
            g_test_failures++;                                                 \
            fprintf(stderr, "FAIL %s:%d: |%g - %g| = %g > %g\n",               \
                    __FILE__, __LINE__, (double)(a), (double)(b), _d,          \
                    (double)(eps));                                            \
        }                                                                      \
    } while (0)

#define TEST_REPORT()                                                          \
    do {                                                                       \
        if (g_test_failures == 0) {                                            \
            printf("PASS: %d checks\n", g_test_checks);                        \
            return 0;                                                          \
        }                                                                      \
        printf("FAIL: %d/%d checks failed\n", g_test_failures, g_test_checks); \
        return 1;                                                              \
    } while (0)

#endif /* TEST_ASSERT_H */
