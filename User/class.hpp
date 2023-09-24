//
// Created by ShiF on 2023/9/19.
//

#ifndef KOSANN_UAVGIMBAL_CLASS_HPP
#define KOSANN_UAVGIMBAL_CLASS_HPP

#include "framework_headfile.hpp"
#include "MCU_heafile.hpp"
#include "core.h"

/*#define _func */
#define YAW 0
#define PITCH 1

class cMotor
{
private:
    uint32_t _std_id;
    CAN_HandleTypeDef *_usehcan;
    uint32_t _can_id;
    Motor_measure_t _can_read;//获取数据,供访问
    PID_t _classic_spd_pid;
    PID_t _classic_pos_pid;
public:
    void canInit(uint32_t can_id_t, uint32_t stdid_t, CAN_HandleTypeDef *);
    void canSend(int16_t current);
    void canRead(uint32_t rx_std_id, uint8_t rx_buffer[8]);
    /*classicPID*/
    void classicPidInit();
    void classicPid(float spd_kp, float spd_ki, float spd_kd, float spd_out_max,
                    float pos_kp, float pos_ki, float pos_kd, float pos_out_max);
    void calcPid(float tar_pos);
    void sendPid();
};

void MotorInit();
extern cMotor yaw;

#endif //KOSANN_UAVGIMBAL_CLASS_HPP
