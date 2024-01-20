//
// Created by ShiF on 2023/9/19.
//

#ifndef KOSANN_UAVGIMBAL_MOTORC_HPP
#define KOSANN_UAVGIMBAL_MOTORC_HPP

#include "framework_headfile.hpp"
#include "MCU_heafile.hpp"
#include "core.h"

class cMotor
{
public:
    typedef enum
    {
        GM_6020_volt,//电压控制
        GM_6020_cur,//电机电流控制
        M_3508,
        M_2006,
        M_3508_p19,
        M_2006_p36
    } eMotorType;   //电机表
    enum eErrorType
    {
        ERROR_MOTOR,
        eCanSend_Error
    };      //错误类型
    typedef enum
    {
        eExternal,  //非控制算法表的算法的额外的控制量输入
        ePid,
        eAdrc
    } eController;  //控制算法
private:
    uint32_t _can_id;
    eMotorType _motor_type;
    eController _controller_type;
    float _ratio;//减速比
    CAN_HandleTypeDef *_usehcan;
    Motor_measure_t _can_read;//获取数据,供访问
    Motor_measure_f _ecd;   //编码器数据
    uint8_t tx_channel;
    uint32_t ID;
public:
    static void errorHandle(eErrorType error_type);
    void canInit(uint32_t can_id_t, CAN_HandleTypeDef *hcan, int motor_type);
    void canSend(eController controller_type, int16_t current);
    void canSend(eController controller_type);
    void canRead(uint32_t rx_std_id, uint8_t rx_buffer[8]);
    Motor_measure_f &getEcd();
    void clearEcdRcnt();
    cFilter autoAim_filter; //自瞄滤波器
    cFilter RCcrtl_filter;  //遥控滤波器
    algorithm MotorCtrl;    //电机控制算法
    void protect();
    bool autoaimflag;
};

class cCar
{
private:
    float _pCarInEcd;//正前方
public:

    typedef enum
    {
        ONMI,
        MECANUM,
        UAV
    } eCarType;//机器人表
    typedef enum
    {
        eRC,
        eRC_Autoaim,
        eKey
    } eCtrlMode;//控制方式
    typedef enum
    {
        eNormal,
        eAutoaim
    } eShootMode;//射击方式


    eCarType CarType;
    eCtrlMode CtrlMode;
    eShootMode ShootMode;
    explicit cCar(eCarType car_type);
    [[nodiscard]] float getPCarInEcd() const;
    void setPCarInEcd(float p_car_in_ecd);
    bool is_protect;    //保护模式变量
};

#endif //KOSANN_UAVGIMBAL_MOTORC_HPP
