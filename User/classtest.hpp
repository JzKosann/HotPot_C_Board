//
// Created by ShiF on 2023/9/19.
//

#ifndef KOSANN_UAVGIMBAL_CLASSTEST_HPP
#define KOSANN_UAVGIMBAL_CLASSTEST_HPP

#include "framework_headfile.hpp"
#include "MCU_heafile.hpp"
#include "core.h"


class motor {
private:
public:
    uint32_t can_ID;
    CAN_HandleTypeDef *usehcan;
    Motor_measure_t can_read;

    void can_Init(uint32_t can_ID, CAN_HandleTypeDef *usehcan);

    void can_send(int16_t current);
};

void motor_Init();

void yaw_cansend(int16_t current);

void motor_read();

#endif //KOSANN_UAVGIMBAL_CLASSTEST_HPP
