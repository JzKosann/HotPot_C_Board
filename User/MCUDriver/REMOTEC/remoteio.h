//
// Created by ShiF on 2023/9/14.
//

#ifndef KOSANN_UAVGIMBAL_REMOTEIO_H
#define KOSANN_UAVGIMBAL_REMOTEIO_H
#include <cstdint>
#include "stm32f4xx_hal.h"

extern void REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
class remoteio {

};

#endif //KOSANN_UAVGIMBAL_REMOTEIO_H
