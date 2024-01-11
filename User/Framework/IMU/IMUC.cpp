//
// Created by ShiF on 2024/1/10.
//

#include "IMUC.hpp"

cimu IMU;

float cimu::cAngle(cimu::eImuType imu_type, cimu::eImuFun imu_fun)
{
    switch (imu_type)
    {
        case C_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return IMU_Angle(0);
                case Pitch:
                    return IMU_Angle(1);
                case Roll:
                    return IMU_Angle(2);
            }
            break;
        case Bno085_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return Bno085_IMU_Angle(0);
                case Pitch:
                    return Bno085_IMU_Angle(1);
                case Roll:
                    return Bno085_IMU_Angle(2);
            }
            break;
        case Wit_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return WIT_IMU_Angle(imu_yaw);
                case Pitch:
                    return WIT_IMU_Angle(imu_pitch);
                case Roll:
                    break;
            }
    }
    return 0;
}

float cimu::cSpeed(cimu::eImuType imu_type, cimu::eImuFun imu_fun)
{
    switch (imu_type)
    {
        case C_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return IMU_Speed(0);
                case Pitch:
                    return IMU_Speed(1);
                case Roll:
                    return IMU_Speed(2);
            }
            break;
        case Bno085_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return Bno085_IMU_Speed(0);
                case Pitch:
                    return Bno085_IMU_Speed(1);
                case Roll:
                    return Bno085_IMU_Speed(2);
            }
            break;
        case Wit_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return WIT_IMU_Speed(imu_yaw);
                case Pitch:
                    return WIT_IMU_Speed(imu_pitch);
                case Roll:
                    break;
            }
            break;
    }
    return 0;
}
