//
// Created by ShiF on 2023/9/19.
//

#ifndef KOSANN_UAVGIMBAL_CLASS_HPP
#define KOSANN_UAVGIMBAL_CLASS_HPP

#include "framework_headfile.hpp"
#include "MCU_heafile.hpp"
#include "core.h"


class motor {
private:
    uint32_t STD_ID;
    CAN_HandleTypeDef *usehcan;
public:
    uint32_t can_ID;
    Motor_measure_t can_read;

    void can_Init(uint32_t canID_t, uint32_t STDID_t, CAN_HandleTypeDef *hcan_t);

    void can_send(int16_t current);
};

void motor_Init();
extern motor yaw;

#endif //KOSANN_UAVGIMBAL_CLASS_HPP
