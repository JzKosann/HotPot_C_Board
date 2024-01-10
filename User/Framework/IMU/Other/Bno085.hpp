//
// Created by ShiF on 2024/1/8.
//

#ifndef KOSANN_UAVGIMBAL_BNO085_HPP
#define KOSANN_UAVGIMBAL_BNO085_HPP
#include "stm32f4xx_hal.h"
void IMU_UartInit(void);
float Bno085_IMU_Angle(uint8_t which);
float Bno085_IMU_Speed(uint8_t which);
#endif //KOSANN_UAVGIMBAL_BNO085_HPP
