/*
 * encoder.c —— 编码器设备装配（按 EncoderType 返回 PositionSensorOps）
 */
#include "device/encoder/encoder.h"

const PositionSensorOps *encoder_ops_for_type(EncoderType type)
{
    switch (type) {
    case ENCODER_TYPE_ABZ:
        return &enc_abi_ops;
    case ENCODER_TYPE_ABS:
        return &enc_abs_ops;
    default:
        return NULL;
    }
}
