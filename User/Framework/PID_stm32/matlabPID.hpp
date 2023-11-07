//
// Created by ShiF on 2023/10/11.
//

#ifndef KOSANN_INFANTRY_GIMBAL_MATLABPID_HPP
#define KOSANN_INFANTRY_GIMBAL_MATLABPID_HPP
#include "PIDC.h"
/**
 * 取自mjw---matlab pid代码
 * 仅有PID POS LOOP
 * 直接赋值 yaw pitch 的目标值和前馈系数
 */
#define mPid_In rtU
#define mPid_Out rtY //matlab生成的PID

typedef class matlabPID{
public:
    void Init();
    void yaw_calc(float yawPosIn,float yawSpdIn);
    void pitch_calc(float pitchPosIn,float pitchSpdIn);
    void yawSetparam(float spd_kp, float spd_ki, float spd_kd, float spd_n, float spd_outmax, float dif_gain,
                     float pos_kp, float pos_ki, float pos_kd, float pos_n, float pos_outmax);
    void pitchSetparam(float spd_kp, float spd_ki, float spd_kd, float spd_outmax,
                     float pos_kp, float pos_ki, float pos_kd, float pos_outmax);
    void yawSetTar(float Tar);
    void pitchSetTar(float Tar);
    int16_t yawOut();
    int16_t pitchOut();
}mPID;

#endif //KOSANN_INFANTRY_GIMBAL_MATLABPID_HPP
