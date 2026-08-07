/*
 * sampling.h —— 采样链路（§4.6，冻结）
 * PWM同步触发 → ADC DMA → 相电流重构(CurrentSense) → SampleFrame → Clarke → FOC
 * 禁止 adc_read() 直给 FOC。
 */
#ifndef FOC_RUNTIME_SAMPLING_H
#define FOC_RUNTIME_SAMPLING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    ia, ib, ic;        /* [A] */
    uint64_t timestamp_us;
    uint32_t cycle;
} SampleFrame;

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_SAMPLING_H */
