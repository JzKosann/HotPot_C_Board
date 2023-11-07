//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_IMUC_H
#define KOSANN_UAVGIMBAL_IMUC_H

#define IMU_DATASIZE 33u

#include "stm32f4xx_hal.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} GetQuaternion; // 0-8191 对应0-360°
typedef enum
{
    imu_yaw,
    imu_pitch
} IMU_data;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} GetNaiveAngle; // 0-8191 对应0-360°

void IMU_UartInit(void);

void IMU_Data_Handler(void);

float IMU_Angle(IMU_data Witch_angle);

float IMU_Speed(IMU_data witch);

float IMU_AngleIncreLoop(float angle_now);

GetQuaternion IMU_Quaternion(void);

GetNaiveAngle IMU_NaiveAngle(void);

#endif //KOSANN_UAVGIMBAL_IMUC_H
