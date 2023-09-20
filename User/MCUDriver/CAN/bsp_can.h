//
// Created by ShiF on 2023/9/14.
//

#ifndef KOSANN_UAVGIMBAL_BSP_CAN_H
#define KOSANN_UAVGIMBAL_BSP_CAN_H

#include "main.h"

extern uint32_t m;
typedef enum {
    First_STDID = 0x200,
    Last_STDID = 0x1FF,

    Chassis_3508A = 0x201,
    Chassis_3508B = 0x202,
    Chassis_3508C = 0x203,
    Chassis_3508D = 0x204,
    CAN1_3508_ID5 = 0x205,
    CAN1_3508_ID6 = 0x206,
    CAN1_6020_ID3 = 0x207,

    CAN2_3508_ID1 = 0x201,
    CAN2_3508_ID2 = 0x202,
    CAN2_3508_ID3 = 0x203,
    CAN2_3508_ID4 = 0x204,
    CAN2_6020_ID1 = 0x205,
    CAN2_6020_ID2 = 0x206,
    CAN2_6020_ID3 = 0x207,
} CAN_ID_e;

typedef struct {
    int16_t speed;
    int16_t torque_current;
    int16_t Output;
    uint8_t temp;
    uint16_t angle;                //abs angle range:[0,8191]
    uint16_t last_angle;            //abs angle range:[0,8191]
    int32_t round_cnt;
    int32_t total_angle;
} Motor_measure_t;

extern Motor_measure_t Motor_measure[14];
extern Motor_measure_t test_motor;

void can_filter_init(CAN_HandleTypeDef *hcan);

void Motor_measure_fun(Motor_measure_t *ptr, uint8_t *RX_buffer);

void Set_motor_cmd(CAN_HandleTypeDef *hcan, uint32_t STDID,
                   int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void Set_motor1_cmd(CAN_HandleTypeDef *hcan, int16_t motor_current);

#endif //KOSANN_UAVGIMBAL_BSP_CAN_H
