//
// Created by ShiF on 2023/9/14.
//

#include "bsp_can.h"
#include "main.h"
#include "MotorC.hpp"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_measure_t motor_measure[14];

void can_filter_init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef can_filter_st;                                                            //定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;                                                    //ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;                                           //设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;                                          //过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;                                                        //ID高位
    can_filter_st.FilterIdLow = 0x0000;                                                         //ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;                                                    //过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;                                                     //过滤器掩码低位
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                                          //与过滤器组管理的 FIFO
    if (hcan->Instance == CAN1) {
        can_filter_st.FilterBank = 0;
    } else if (hcan->Instance == CAN2) {
        can_filter_st.FilterBank = 14;
    }
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);                                     //HAL库配置过滤器函数
    HAL_CAN_Start(hcan);                                                                            //使能CAN控制器
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);                    //使能CAN的各种中断
}

void MotorMeasureFun(Motor_measure_t *ptr, uint8_t *RX_buffer) {
    ptr->last_angle = ptr->angle;                                                       //记录上一次转子机械角度
    ptr->angle = (int16_t) ((RX_buffer)[0] << 8 | (RX_buffer)[1]);                     //解析转子机械角度
    ptr->speed = (int16_t) ((RX_buffer)[2] << 8 | (RX_buffer)[3]);                     //解析转子转速(rpm)
    ptr->torque_current = (int16_t) ((RX_buffer)[4] << 8 | (RX_buffer)[5]);            //解析转矩电流
    ptr->temp = (RX_buffer)[6];

    if (ptr->angle - ptr->last_angle > 4096)                                        //利用转子本次机械角度与上次机械角度的差值
        ptr->round_cnt--;                                                           //判断电机转子正向或反向过零
    else if (ptr->angle - ptr->last_angle < -4096)                                  //从而计算得到转子从上电开始转动的总圈数
        ptr->round_cnt++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;                          //利用总圈数计算转子总角度
                                                                                    //(出轴总角度要根据减速比再换算
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;                                                                //定义数据帧的帧头
    uint8_t rx_buffer[8];                                                                         //接收存放数据帧数据的数组

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer);         //把can接收到的数据帧传入局部变量

    static uint8_t i = 0;
    if (hcan == &hcan1) {
        i = rx_header.StdId - Chassis_3508A;//通过反馈数据的ID确定这一组数据存放的地址
        MotorMeasureFun(&motor_measure[i], rx_buffer);                                           //调用函数把数据存入结构体数组

    } else if (hcan == &hcan2) {                //can2
        yaw.canRead(rx_header.StdId,rx_buffer);
        pitch.canRead(rx_header.StdId,rx_buffer);
    }
}


