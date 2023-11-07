//
// Created by ShiF on 2023/9/19.
//

#include "MotorC.hpp"

void cMotor::errorHandle(eErrorType error_type)
{
    while (1)
    {
        switch (error_type)
        {
            case ERROR_MOTOR:
                usart_printf("motor setting error\r\n");
                break;
            case eCanSend_Error:
                usart_printf("can send error\r\n");
                break;
        }
    }
}

/**
 * can串口发送电流
 * @param current 电流
 */
void cMotor::canSend( eController controller_type,int16_t current)
{

    _controller_type = (eController) controller_type;
    if(_controller_type!=eExternal){
        errorHandle(eCanSend_Error);
    }else{
    CanReg[tx_channel].CANx_send_data[(ID - 1) * 2] = current >> 8;
    CanReg[tx_channel].CANx_send_data[(ID - 1) * 2 + 1] = current;
    }
}

void cMotor::canSend(eController controller_type)
{
    int16_t current=0;
    _controller_type = (eController) controller_type;
    switch (_controller_type)
    {
        case eExternal:
            errorHandle(eCanSend_Error);
            break;
        case ePid:
            current = (int16_t) MotorCtrl.c_PID.Pid_Out();
            break;
        case eAdrc:
            current=(int16_t)MotorCtrl.c_ADRC.ADRCout();
            break;
    }
    CanReg[tx_channel].CANx_send_data[(ID - 1) * 2] = current >> 8;
    CanReg[tx_channel].CANx_send_data[(ID - 1) * 2 + 1] = current;
}

void cMotor::canInit(uint32_t can_id_t, CAN_HandleTypeDef *hcan_t, int motor_type)
{
    _can_id = can_id_t;
    _usehcan = hcan_t;
    _motor_type = (eMotorType) motor_type;
    ID = _can_id - 0x200;
    switch (_motor_type)
    {
        case GM_6020:
            if (_usehcan == &hcan1)
            {
                tx_channel = (ID <= 8) ? 0 : 2;
            }
            else if (_usehcan == &hcan2)
            {
                tx_channel = (ID <= 8) ? 3 : 5;
            }
            ID -= 4;
            break;
        case M_3508:
            if (_usehcan == &hcan1)
            {
                tx_channel = (ID <= 4) ? 1 : 0;
            }
            else if (_usehcan == &hcan2)
            {
                tx_channel = (ID <= 4) ? 4 : 3;
            }
            break;
        case M_2006:
            if (_usehcan == &hcan1)
            {
                tx_channel = (ID <= 4) ? 1 : 0;
            }
            else if (_usehcan == &hcan2)
            {
                tx_channel = (ID <= 4) ? 4 : 3;
            }
            break;
        case M_3508_p19:
            if (_usehcan == &hcan1)
            {
                tx_channel = (ID <= 4) ? 1 : 0;
            }
            else if (_usehcan == &hcan2)
            {
                tx_channel = (ID <= 4) ? 4 : 3;
            }
            break;
        case M_2006_p36:
            if (_usehcan == &hcan1)
            {
                tx_channel = (ID <= 4) ? 1 : 0;
            }
            else if (_usehcan == &hcan2)
            {
                tx_channel = (ID <= 4) ? 4 : 3;
            }
            break;
        default:
            errorHandle(ERROR_MOTOR);
            break;
    }
    if (ID >= 5) ID -= 4;

    switch (_motor_type)
    {

        case GM_6020:
            _ratio = 1;
            break;
        case M_3508:
            _ratio = 1;
            break;
        case M_2006:
            _ratio = 1;
            break;
        case M_3508_p19:
            _ratio = 19;
            break;
        case M_2006_p36:
            _ratio = 36;
            break;
        default:
            errorHandle(ERROR_MOTOR);
            break;
    }
    CanReg[tx_channel].send_flag = 1;
}

void cMotor::canRead(uint32_t rx_std_id, uint8_t rx_buffer[8])
{
    if (_can_id == rx_std_id) MotorMeasureFun(&_can_read, rx_buffer);
}

void cMotor::protect()
{
    canSend(eExternal,0);
}

Motor_measure_f &cMotor::getEcd()
{
    _ecd.total_angle = (float) _can_read.total_angle * 360 / 8191;
    _ecd.speed = (float) _can_read.speed / _ratio;
    _ecd.angle = (float) _can_read.angle * 360 / 8191;
    _ecd.torque_current = (float) _can_read.torque_current;
    _ecd.temp = (float) _can_read.temp;
    _ecd.Output = (float) _can_read.Output;
    return _ecd;
}

void cMotor::clearEcdRcnt()
{
    _can_read.round_cnt = 0;

}

float cCar::getPCarInEcd() const
{
    return _pCarInEcd;
}


cCar::cCar(cCar::eCarType car_type) : CarType(car_type)
{
    switch (CarType)
    {

        case ONMI:
            _pCarInEcd = 347;
            break;
        case MECANUM:
            break;
        case UAV:
            break;
    }
}

void cCar::setPCarInEcd(float p_car_in_ecd)
{
    _pCarInEcd += p_car_in_ecd;
}
