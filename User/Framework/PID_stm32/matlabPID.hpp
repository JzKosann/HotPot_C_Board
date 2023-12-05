//
// Created by ShiF on 2023/10/11.
//

#ifndef KOSANN_INFANTRY_GIMBAL_MATLABPID_HPP
#define KOSANN_INFANTRY_GIMBAL_MATLABPID_HPP
#include "PID_mat.hpp"
#include "PosPID.hpp"
/**
 * 取自mjw---matlab pid代码
 * 仅有PID POS LOOP
 * 直接赋值 yaw pitch 的目标值和前馈系数
 */
#define SpdPid_In PID_mat_U
#define SpdPid_Out PID_mat_Y //matlab生成的PID
#define PosPid_In PosPID_U
#define PosPid_Out PosPID_Y //matlab生成的PID


typedef class matlabPID{
public:
    void Init();
    void calc(float input);
    void settar(float tar);
    void setPara(float Kp,float Ki,float OUTMAX);
    void PosSetPara(float Kp,float Ki,float Kd,float Kn,float OUTMAX);
    void PosLoop(float input,float tar);
    float PosOut();
    float out();
}mPID;

#endif //KOSANN_INFANTRY_GIMBAL_MATLABPID_HPP
