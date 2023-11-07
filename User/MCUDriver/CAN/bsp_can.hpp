//
// Created by ShiF on 2023/9/14.
//

#ifndef KOSANN_UAVGIMBAL_BSP_CAN_H
#define KOSANN_UAVGIMBAL_BSP_CAN_H

#include "main.h"

extern uint32_t m;

typedef struct
{
    int16_t speed;
    int16_t torque_current;
    int16_t Output;
    uint8_t temp;
    uint16_t angle;                //abs angle range:[0,8191]
    uint16_t last_angle;            //abs angle range:[0,8191]
    int32_t round_cnt;
    int32_t total_angle;
} Motor_measure_t;
typedef struct
{
    float speed;
    float torque_current;
    float Output;
    float temp;
    float angle;                //abs angle range:[0,8191]
    float total_angle;
} Motor_measure_f;


void can_filter_init(CAN_HandleTypeDef *hcan);

void MotorMeasureFun(Motor_measure_t *ptr, uint8_t *RX_buffer);


class CanType
{
private:
    CAN_HandleTypeDef *_hcan;
    uint32_t _std_id;
    CAN_TxHeaderTypeDef CANx_tx_message;
    uint32_t send_mail_box;
public:
    uint8_t send_flag;      //确定是否发送
    uint8_t CANx_send_data[8];
    CanType(CAN_HandleTypeDef *hcan, uint32_t std_id);
    void sendBuff();
};

extern CanType CanReg[];
#endif //KOSANN_UAVGIMBAL_BSP_CAN_H
