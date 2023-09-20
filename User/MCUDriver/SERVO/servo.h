//
// Created by ShiF on 2023/9/14.
//

#ifndef KOSANN_UAVGIMBAL_SERVO_H
#define KOSANN_UAVGIMBAL_SERVO_H
#include "stm32f4xx_hal.h"
void Servo_on(void);
void Servo_off(void);
void Servo_StartPWM(void);
int8_t Servo_GetStatus(void);
#endif //KOSANN_UAVGIMBAL_SERVO_H
