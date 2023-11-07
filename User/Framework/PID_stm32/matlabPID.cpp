//
// Created by ShiF on 2023/10/11.
//
#include "matlabPID.hpp"

void matlabPID::Init()
{
    PID_initialize();
}

void matlabPID::yaw_calc(float yawPosIn,float yawSpdIn)
{
    mPid_In.YawAngle_Now = yawPosIn;
    mPid_In.YawSpeed_Now = yawSpdIn;
    PID_step(1);
}

void matlabPID::pitch_calc(float pitchPosIn,float pitchSpdIn)
{

}

void matlabPID::yawSetparam(float spd_kp, float spd_ki, float spd_kd, float spd_n, float spd_outmax, float dif_gain,
                            float pos_kp, float pos_ki, float pos_kd, float pos_n, float pos_outmax)
{
    mPid_In.YawP_P = pos_kp;
    mPid_In.YawP_I = pos_ki;
    mPid_In.YawP_D = pos_kd;
    mPid_In.YawP_N = pos_n;
    mPid_In.YawP_MO = pos_outmax;
    mPid_In.YawP_LO = -mPid_In.YawP_MO;
    mPid_In.YawS_P = spd_kp;
    mPid_In.YawS_I = spd_ki;
    mPid_In.YawS_D = spd_kd;
    mPid_In.YawS_N = spd_n;
    mPid_In.YawS_MO = spd_outmax;
    mPid_In.YawS_LO = -mPid_In.YawS_MO;
    mPid_In.Yaw_Dif_Gain = dif_gain;
}

void matlabPID::pitchSetparam(float spd_kp, float spd_ki, float spd_kd, float spd_outmax, float pos_kp, float pos_ki, float pos_kd, float pos_outmax)
{

}

void matlabPID::yawSetTar(float Tar)
{
    mPid_In.YawAngle_set = Tar;
}

void matlabPID::pitchSetTar(float Tar)
{

}

int16_t matlabPID::yawOut()
{
    return mPid_Out.YawCurrent;
}

int16_t matlabPID::pitchOut()
{
    return mPid_Out.PihCurrent;
}



