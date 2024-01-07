//
// Created by ShiF on 2023/10/6.
//

#ifndef KOSANN_INFANTRY_GIMBAL_GIMBALC_HPP
#define KOSANN_INFANTRY_GIMBAL_GIMBALC_HPP

#include "framework_headfile.hpp"
#include "MCU_heafile.hpp"
#include "core.h"
#include "MotorC.hpp"
extern CanType CanReg[];

void Gimbal_Init();

void GimbalLoop();
bool portSetProtect();
void portSetChassicStop();

/**external value**/
extern cMotor yaw;
extern cMotor pitch;
extern cMotor fricR, fricL;
extern cMotor rammc;

/**com define**/
#define ChassisVel CanReg[6]
#define ChassisYaw CanReg[7]
/**car mode**/
//#define

#endif //KOSANN_INFANTRY_GIMBAL_GIMBALC_HPP
