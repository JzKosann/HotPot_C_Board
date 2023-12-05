//
// Created by ShiF on 2023/10/11.
//
#include "matlabPID.hpp"

void matlabPID::Init()
{
    PID_mat_initialize();
}

void matlabPID::calc(float input)
{
    SpdPid_In.Now=input;
    PID_mat_step();
}
float matlabPID::out(){

    return SpdPid_Out.Current;
}

void matlabPID::settar(float tar)
{
    SpdPid_In.Tar=tar;
}

void matlabPID::setPara(float Kp, float Ki, float OUTMAX)
{
    SpdPid_In.Kp=Kp;
    SpdPid_In.Ki=Ki;
    SpdPid_In.OUTMAX=OUTMAX;
    SpdPid_In.OUTLOW=-OUTMAX;
}

void matlabPID::PosLoop(float input, float tar)
{
    PosPid_In.Tar=tar;
    PosPid_In.Now=input;
    PosPID_step();
}

float matlabPID::PosOut()
{
    return PosPid_Out.spd;
}

void matlabPID::PosSetPara(float Kp, float Ki, float Kd, float Kn, float OUTMAX)
{
    PosPid_In.Kp=Kp;
    PosPid_In.Ki=Ki;
    PosPid_In.Kd=Kd;
    PosPid_In.N=Kn;
    PosPid_In.OUTMAX=OUTMAX;
    PosPid_In.OUTLOW=-OUTMAX;
}
