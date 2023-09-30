//
// Created by ShiF on 2023/9/19.
//

#include "MotorC.hpp"

static CAN_TxHeaderTypeDef CANx_tx_message;
static uint8_t CANx_send_data[8];

/**电机列表**/
cMotor yaw, pitch;          //云台功能


void MotorInit()
{
    yaw.canInit(0x209, &hcan2, GM6020);
    pitch.canInit(0x209, &hcan2, GM6020);
    /*PID*/
    yaw.classicPidInit();
    pitch.classicPidInit();
    yaw.classicPid(0,0,0,0,0,0,0,0);
    pitch.classicPid(35, 0.2, 0, 2000, 1, 0, 0.3, 300);
}

void cMotor::errorHandle(eRrorType error_type)
{
    while (1)
    {
        switch (error_type)
        {
            case ERROR_MOTOR:
                usart_printf("motor setting error\r\n");
                break;
        }
    }
}

/**
 * can串口发送电流
 * @param current 电流
 */
void cMotor::canSend(int16_t current)
{
    uint32_t send_mail_box;
    CANx_tx_message.StdId = _std_id;
    CANx_tx_message.IDE = CAN_ID_STD;                                                             //标识符选择位，STD-标准帧
    CANx_tx_message.RTR = CAN_RTR_DATA;                                                           //定义帧类型
    CANx_tx_message.DLC = 0x08;
    switch (_motor_type)
    {
        case GM_6020:
            switch (_can_id)
            {
                case 0x205:
                    CANx_send_data[0] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[1] = current;
                    break;
                case 0x206:
                    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[3] = current;
                    break;
                case 0x207:
                    CANx_send_data[4] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[5] = current;
                    break;
                case 0x208:
                    CANx_send_data[6] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[7] = current;
                    break;
                case 0x209:
                    CANx_send_data[0] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[1] = current;
                    break;
                case 0x20A:
                    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[3] = current;
                    break;
                case 0x20B:
                    CANx_send_data[4] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[5] = current;
                    break;
                default:
                    break;
            }
            break;
        case M_3508:
            switch (_can_id)
            {
                case 0x201:
                    CANx_send_data[0] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[1] = current;
                    break;
                case 0x202:
                    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[3] = current;
                    break;
                case 0x203:
                    CANx_send_data[4] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[5] = current;
                    break;
                case 0x204:
                    CANx_send_data[6] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[7] = current;
                    break;
                case 0x205:
                    CANx_send_data[0] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[1] = current;
                    break;
                case 0x206:
                    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[3] = current;
                    break;
                case 0x207:
                    CANx_send_data[4] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[5] = current;
                    break;
                case 0x208:
                    CANx_send_data[6] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[7] = current;
                    break;
                default:
                    break;
            }
            break;
        case M_2006:
            switch (_can_id)
            {
                case 0x201:
                    CANx_send_data[0] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[1] = current;
                    break;
                case 0x202:
                    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[3] = current;
                    break;
                case 0x203:
                    CANx_send_data[4] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[5] = current;
                    break;
                case 0x204:
                    CANx_send_data[6] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[7] = current;
                    break;
                case 0x205:
                    CANx_send_data[0] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[1] = current;
                    break;
                case 0x206:
                    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[3] = current;
                    break;
                case 0x207:
                    CANx_send_data[4] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[5] = current;
                    break;
                case 0x208:
                    CANx_send_data[6] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
                    CANx_send_data[7] = current;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }


    HAL_CAN_AddTxMessage(_usehcan,
                         &CANx_tx_message,                           //hal库can发送函数：该函数用于向发送邮箱
                         CANx_send_data, &send_mail_box);            //添加发送报文，并激活发送请求
}

void cMotor::canInit(uint32_t can_id_t, CAN_HandleTypeDef *hcan_t, int motor_type)
{
    _can_id = can_id_t;
    _usehcan = hcan_t;
    _motor_type = (eMotorType) motor_type;
    switch (_motor_type)
    {
        case GM_6020:
            if (_can_id >= 0x205 && _can_id <= 0x208) _std_id = 0x1FF;
            else if (_can_id >= 0x209 && _can_id <= 0x20B) _std_id = 0x2FF;
            else errorHandle(ERROR_MOTOR);
            break;
        case M_3508:
            if (_can_id >= 0x201 && _can_id <= 0x204) _std_id = 0x200;
            else if (_can_id >= 0x205 && _can_id <= 0x208) _std_id = 0x1FF;
            else errorHandle(ERROR_MOTOR);
            break;
        case M_2006:
            if (_can_id >= 0x201 && _can_id <= 0x204) _std_id = 0x200;
            else if (_can_id >= 0x205 && _can_id <= 0x208) _std_id = 0x1FF;
            else errorHandle(ERROR_MOTOR);
        default:
            break;
    }
}

void cMotor::canRead(uint32_t rx_std_id, uint8_t rx_buffer[8])
{
    if (_can_id == rx_std_id) MotorMeasureFun(&_can_read, rx_buffer);
}

/**     classic-PID     **/
void cMotor::classicPidInit()
{
    PID_SpeedParamInit(&_classic_spd_pid);
    PID_PosParamInit(&_classic_pos_pid);
}

void cMotor::classicPid(float spd_kp, float spd_ki, float spd_kd, float spd_out_max, float pos_kp, float pos_ki, float pos_kd, float pos_out_max)
{
    _classic_pos_pid.Kp1 = pos_kp;
    _classic_pos_pid.Ki1 = pos_ki;
    _classic_pos_pid.Kd1 = pos_kd;
    _classic_pos_pid.PID_OutMax = pos_out_max;                  //带载时电机速度不超过200RPM
    _classic_spd_pid.Kp1 = spd_kp;
    _classic_spd_pid.Ki1 = spd_ki;
    _classic_spd_pid.Kd1 = spd_kd;
    _classic_spd_pid.PID_OutMax = spd_out_max;
}

void cMotor::calcPid(float tar_pos)
{
    _classic_pos_pid.PID_Target = Debug_Param().vel_rampTargetValue;
    PID_Update(&_classic_pos_pid, (float) _can_read.total_angle);
    PID_GetPositionPID(&_classic_pos_pid);
    _classic_spd_pid.PID_Target = _classic_pos_pid.PID_Out;
    PID_Update(&_classic_spd_pid, _can_read.speed);
    PID_GetPositionPID(&_classic_spd_pid);
}

void cMotor::sendPid()
{
    canSend(_classic_spd_pid.PID_Out);
}

/**     ADRC        **/
