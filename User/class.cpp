//
// Created by ShiF on 2023/9/19.
//

#include "class.hpp"

static CAN_TxHeaderTypeDef CANx_tx_message;
static uint8_t CANx_send_data[8];

motor yaw;

void motor_Init() {
    yaw.can_Init(0x205, 0x1FF, &hcan2);
}

/**
 * can串口发送电流
 * @param current 电流
 */
void motor::can_send(int16_t current) {

    uint32_t send_mail_box;
    CANx_tx_message.StdId = STD_ID;
    CANx_tx_message.IDE = CAN_ID_STD;                                                             //标识符选择位，STD-标准帧
    CANx_tx_message.RTR = CAN_RTR_DATA;                                                           //定义帧类型
    CANx_tx_message.DLC = 0x08;
    switch (STD_ID) {
        case 0x1FF:
            switch (can_ID) {
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
        case 0x2FF:
            switch (can_ID) {
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
                default:
                    break;
            }
            break;
    }

    HAL_CAN_AddTxMessage(usehcan,
                         &CANx_tx_message,                           //hal库can发送函数：该函数用于向发送邮箱
                         CANx_send_data, &send_mail_box);            //添加发送报文，并激活发送请求
}

void motor::can_Init(uint32_t canID_t, uint32_t STDID_t, CAN_HandleTypeDef *hcan_t) {
    STD_ID = STDID_t;
    can_ID = canID_t;
    usehcan = hcan_t;
}

