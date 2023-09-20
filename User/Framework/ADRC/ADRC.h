//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_ADRC_H
#define KOSANN_UAVGIMBAL_ADRC_H
#include "math.h"
typedef struct {
    float x1;
    float x2;
    float z1;
    float z2;
    float z3;
    float target;
    float feedback;
    float delta;
    float u0;
    float u;
    //TD
    float r;
    float h;

    //ESO
    float epsilon;
    float b;
    float beta01;
    float beta02;
    float beta03;

    //非线性反馈
    float alpha1;
    float alpha2;
    float betac1;
    float betac2;
} ADRC_t;

void ADRCparamInit(ADRC_t *which);
float ADRC_calc(ADRC_t *which, float target, float feedback);

#endif //KOSANN_UAVGIMBAL_ADRC_H
