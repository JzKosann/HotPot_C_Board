
#ifndef KOSANN_UAVGIMBAL_FILTER_HPP
#define KOSANN_UAVGIMBAL_FILTER_HPP

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus

#include <algorithm>
#include <string.h>
#include "kalman.hpp"
#include <cmath>

class LowPassFilter
{
private:
    double k;   // 滤波器系数
    double y;   // 上一时刻的输出值
public:
    void Init(double freq, double dt)
    {
        double rc = 1.0 / (2 * M_PI * freq);
        k = dt / (rc + dt);
        y = 0.0;
    }

    double filter(double x)
    {
        y = k * x + (1 - k) * y;
        return y;
    }
};

/*filter-set--------------------------------------------------*/
class cFilter
{
public:
    kalman ckalman;
    LowPassFilter cLowPass;
};

#endif
#endif //KOSANN_UAVGIMBAL_FILTER_HPP
