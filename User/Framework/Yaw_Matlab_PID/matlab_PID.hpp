//
// Created by ShiF on 2024/1/7.
//

#ifndef KOSANN_UAVGIMBAL_MATLAB_PID_HPP
#define KOSANN_UAVGIMBAL_MATLAB_PID_HPP

#include "SpdLoop_Matlabsimulink.hpp"
#include "PosLoop_Matlabsimulink.hpp"

class Mat_Pid
{
private:


public:
    float _pos_forward;     //αǰ��ϵ��
    void Init();            //��ʼ�� ��ʵʲô��û��
    void Calc(float SpdInput, float PosInput);   //���� ���봫��������ֵ
    void SetTar(float tar);                     //����λ��Ŀ��ֵ
    float Out();            //�㷨���
    void SetPara(float Spd_Kp, float Spd_Ki, float Spd_Kd, float Spd_Kn, float Spd_Outmax,
                 float Pos_Kp, float Pos_Ki, float Pos_Kd, float Pos_Kn, float Pos_Outmax, float pos_forward);

};


extern Mat_Pid yaw_mat;
#endif //KOSANN_UAVGIMBAL_MATLAB_PID_HPP
