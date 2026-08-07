/*
 * calibration.h —— 校准插件化（§9.2，冻结）
 * ABZ 与 SPI 两个实现才抽象；V0.1 提供最小占位（encoder/phase 空实现）。
 */
#ifndef FOC_CONTROL_CALIBRATION_H
#define FOC_CONTROL_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int (*encoder)(void *ctx);
    int (*phase)(void *ctx);
    int (*current)(void *ctx);
} CalibrationOps;

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_CALIBRATION_H */
