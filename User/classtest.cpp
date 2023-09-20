//
// Created by ShiF on 2023/9/19.
//

#include "classtest.hpp"

static CAN_TxHeaderTypeDef CANx_tx_message;
static uint8_t CANx_send_data[8];
motor yaw;

void motor_Init() {

    yaw.can_Init(0x1FF, &hcan2);
}

void motor::can_send(int16_t current) {
    uint32_t send_mail_box;
    CANx_tx_message.StdId = can_ID;
    CANx_tx_message.IDE = CAN_ID_STD;                                                             //标识符选择位，STD-标准帧
    CANx_tx_message.RTR = CAN_RTR_DATA;                                                           //定义帧类型
    CANx_tx_message.DLC = 0x08;
    CANx_send_data[0] = 0 >> 8;                                     //依次将要发送的数据移入数据数组，下同
    CANx_send_data[1] = 0;
    CANx_send_data[2] = current >> 8;                               //依次将要发送的数据移入数据数组，下同
    CANx_send_data[3] = current;
    HAL_CAN_AddTxMessage(usehcan,
                         &CANx_tx_message,                           //hal库can发送函数：该函数用于向发送邮箱
                         CANx_send_data, &send_mail_box);            //添加发送报文，并激活发送请求

}

void motor::can_Init(uint32_t canID, CAN_HandleTypeDef *hcan) {
    can_ID = canID;
    usehcan = hcan;
}

void yaw_cansend(int16_t current) {
    yaw.can_send(current);
}

void motor_read() {
    uint8_t RX_BUFFER[8];
    Motor_measure_fun(&yaw.can_read,RX_BUFFER);
}
