//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_DEBUGC_H
#define KOSANN_UAVGIMBAL_DEBUGC_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

#define DEBUG_RVSIZE 255
typedef struct {
    float vel_kp;  //50
    float vel_ki;
    float vel_kd;
    int32_t vel_maxOutput;
    int32_t vel_maxIntegral;
    int32_t vel_rampTargetValue;
    int32_t vel_rampTargetTime;
    int32_t vel_rampTargetStep;

    float pos_kp;
    float pos_ki;
    float pos_kd;
    int32_t pos_maxOutput;
    int32_t pos_maxIntegral;  //800
    int32_t pos_maxOutStep;    //改大了速度跟不上   就是这个问题 导致速度跟不上，到达目标位置附近小幅度震荡（正负3左右）
    int32_t pos_targetAngle;
} DebugParam;

#define STARTPID  0x70         //start_pid
#define STARTLQR 0x6C          //start_lqr
#define START 0x31
#define STOP  0x30
#define MAOHAO  0x3A

#define VEL_LOOP 0x73
#define VEL_KP 0x70            //s_kp
#define VEL_KI 0x69            //s_ki
#define VEL_KD 0x64            //s_kd
#define VEL_MAXOUT 0x6F        //s_mo
#define VEL_MAXINTEGRAL 0x61   //s_ma
#define VEL_TARVALUE 0x76      //s_tv
#define VEL_TARTIME 0x74       //s_tt
#define VEL_TARSTEP 0x73       //s_ts

#define POS_LOOP 0x70
#define POS_KP 0x70            //p_kp
#define POS_KI 0x69            //p_ki
#define POS_KD 0x64            //p_kd
#define POS_MAXOUT 0x6F        //p_mo
#define POS_MAXINTEGRAL 0x61   //p_ma
#define POS_MAXSTEP 0x73       //p_ms
#define POS_TARVALUE 0x76      //p_tv

DebugParam Debug_Param();
void usart_printf(const char *format, ...);
void DEBUGC_UartInit(void);

#endif //KOSANN_UAVGIMBAL_DEBUGC_H
