//
// Created by ShiF on 2024/1/7.
//

#include "matlab_PID.hpp"
#include "SpdLoop_Matlabsimulink.hpp"
#include "PosLoop_Matlabsimulink.hpp"
#include "DebugC.h"

Mat_Pid yaw_mat;
void Mat_Pid::Init()
{
    SpdLoop_Matlabsimulink_initialize();
}

void Mat_Pid::SetTar(float tar)
{
    PosLoop_Matlabsimulink_U.Pos_Tar=tar;
}

void Mat_Pid::Calc(float SpdInput, float PosInput)
{
    PosLoop_Matlabsimulink_U.Pos_now=PosInput;
    SpdLoop_Matlabsimulink_U.Spd_now=SpdInput;

    PosLoop_Matlabsimulink_step();
    SpdLoop_Matlabsimulink_U.Spd_Tar=PosLoop_Matlabsimulink_Y.Pos_Output;
    SpdLoop_Matlabsimulink_step();
    usart_printf("%.2f\r\n", PosLoop_Matlabsimulink_Y.Pos_Output);//打印数据到调参助手
}

float Mat_Pid::Out()
{
    return SpdLoop_Matlabsimulink_Y.Spd_Output;
}

void Mat_Pid::SetPara(float Spd_Kp,float Spd_Ki,float Spd_Kd,float Spd_Kn,float Spd_Outmax,
                      float Pos_Kp,float Pos_Ki,float Pos_Kd,float Pos_Kn,float Pos_Outmax)
{
    SpdLoop_Matlabsimulink_U.Spd_Kp=Spd_Kp;
    SpdLoop_Matlabsimulink_U.Spd_Ki=Spd_Ki;
    SpdLoop_Matlabsimulink_U.Spd_Kd=Spd_Kd;
    SpdLoop_Matlabsimulink_U.Spd_Kn=Spd_Kn;
    SpdLoop_Matlabsimulink_U.Spd_OUTLOW=-Spd_Outmax;
    SpdLoop_Matlabsimulink_U.Spd_OUTMAX=Spd_Outmax;
    PosLoop_Matlabsimulink_U.Pos_Kp=Pos_Kp;
    PosLoop_Matlabsimulink_U.Pos_Ki=Pos_Ki;
    PosLoop_Matlabsimulink_U.Pos_Kd=Pos_Kd;
    PosLoop_Matlabsimulink_U.Pos_Kn=Pos_Kn;
    PosLoop_Matlabsimulink_U.Pos_OUTMAX=Pos_Outmax;
    PosLoop_Matlabsimulink_U.Pos_OUTLOW=-Pos_Outmax;
}
