//
// Created by ShiF on 2024/1/7.
//

#ifndef KOSANN_UAVGIMBAL_MATLAB_PID_HPP
#define KOSANN_UAVGIMBAL_MATLAB_PID_HPP
class Mat_Pid{
private:


public:
    float _pos_forward;
    void Init();
    void Calc(float SpdInput,float PosInput);
    void SetTar(float tar);
    float Out();
    void SetPara(float Spd_Kp,float Spd_Ki,float Spd_Kd,float Spd_Kn,float Spd_Outmax,
                 float Pos_Kp,float Pos_Ki,float Pos_Kd,float Pos_Kn,float Pos_Outmax,float pos_forward);

};



extern Mat_Pid yaw_mat;
#endif //KOSANN_UAVGIMBAL_MATLAB_PID_HPP
