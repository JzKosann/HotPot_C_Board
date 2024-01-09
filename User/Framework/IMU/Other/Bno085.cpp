//
// Created by ShiF on 2024/1/8.
//

#include "Bno085.hpp"
#include "usart.h"
#include <cstring>

static float Yaw_Angle, Yaw_Speed, Pih_Angle, Pih_Speed, Roll_Angle, Roll_Speed;
static float last_angle;
static int32_t rotate_times;
#define IMU_DATASIZE 19u
#define IMU_RVSIZE 255

char IMURvBuff[IMU_RVSIZE] = {0};  //存放串口（调试用）接收的第一手数据
char IMUBuff[IMU_RVSIZE] = {0};    //进行一定变换
char *imupEnd;


void IMU_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *) IMURvBuff, IMU_RVSIZE);
}

void IMU_Data_Handler(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(huart);
    memcpy(IMUBuff, &IMURvBuff[0], IMU_DATASIZE);
    uint8_t data_length = IMU_RVSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);   //计算接收到的数据长度

    if (IMUBuff[0] == 0xAA && IMUBuff[1] == 0xAA)
    {
        Yaw_Angle = (float) ((int16_t) (IMUBuff[3] | (IMUBuff[4] << 8))) * 0.01f;
        Pih_Angle = (float) ((int16_t) (IMUBuff[5] | (IMUBuff[6] << 8))) * 0.01f;
        Roll_Angle = (float) ((int16_t) (IMUBuff[7] | (IMUBuff[8] << 8))) * 0.01f;
        Yaw_Speed = (float) ((int16_t) (IMUBuff[9] | (IMUBuff[10] << 8))) * 0.01f;
        Pih_Speed = (float) ((int16_t) (IMUBuff[11] | (IMUBuff[12] << 8))) * 0.01f;
        Roll_Speed = (float) ((int16_t) (IMUBuff[13] | (IMUBuff[14] << 8))) * 0.01f;
    }


    memset(IMURvBuff, 0, 33);
    HAL_UART_Receive_DMA(huart, (uint8_t *) IMURvBuff, IMU_DATASIZE);


}


void IMU_Receive_Data(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        IMU_Data_Handler(&huart1);

    }
}