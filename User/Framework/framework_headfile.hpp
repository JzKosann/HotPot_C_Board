//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_FRAMEWORK_H
#define KOSANN_UAVGIMBAL_FRAMEWORK_H

#include "ADRC.hpp"
#include "DebugC.h"
#include "filter.hpp"
#include "kalman.hpp"
#include "iwdgC.h"
#include "LEDC.h"
#include "pidC.hpp"
#include "RemoteC.h"
#include "lrgb.h"
#include "packet.hpp"
#include "matlabPID.hpp"
#include "refee.hpp"
#include "../../User/Framework/IMU/bsp/bsp_delay.h"

class algorithm
{
public:
    cPID c_PID;
    cADRC c_ADRC;
};


#endif //KOSANN_UAVGIMBAL_FRAMEWORK_H
