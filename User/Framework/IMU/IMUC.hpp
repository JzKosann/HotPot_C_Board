//
// Created by ShiF on 2024/1/10.
//

#ifndef KOSANN_UAVGIMBAL_IMUC_HPP
#define KOSANN_UAVGIMBAL_IMUC_HPP

#include "framework_headfile.hpp"
#include "Bno085.hpp"
#include "INS_task.h"
#include "filter.hpp"
#include "witreg.hpp"


class cimu
{
private:

public:
    typedef enum
    {
        C_imu,
        Bno085_imu,
        Wit_imu
    } eImuType;

    typedef enum
    {
        Yaw,
        Pitch,
        Roll
    } eImuFun;

    eImuType ImuType;
    eImuFun ImuFun;
    float cAngle(eImuType imu_type, eImuFun imu_fun);
    float cSpeed(eImuType imu_type, eImuFun imu_fun);
    cFilter yaw_filter;
    cFilter pitch_filter;
    cFilter roll_filter;
};

extern cimu IMU;
#endif //KOSANN_UAVGIMBAL_IMUC_HPP
