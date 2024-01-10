//
// Created by ShiF on 2024/1/8.
//

#include "Bno085.hpp"
#include "usart.h"
#include <cstring>

static float angle[3]={};
static float Yaw_Angle, Yaw_Speed, Pih_Angle, Pih_Speed, Roll_Angle, Roll_Speed;
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
        angle[0] = (float) ((int16_t) (IMUBuff[3] | (IMUBuff[4] << 8))) * 0.01f;
        angle[1] = (float) ((int16_t) (IMUBuff[5] | (IMUBuff[6] << 8))) * 0.01f;
        angle[2] = (float) ((int16_t) (IMUBuff[7] | (IMUBuff[8] << 8))) * 0.01f;
        Yaw_Speed = (float) ((int16_t) (IMUBuff[9] | (IMUBuff[10] << 8))) * 0.01f;
        Pih_Speed = (float) ((int16_t) (IMUBuff[11] | (IMUBuff[12] << 8))) * 0.01f;
        Roll_Speed = (float) ((int16_t) (IMUBuff[13] | (IMUBuff[14] << 8))) * 0.01f;
    }


    memset(IMURvBuff, 0, 33);
    HAL_UART_Receive_DMA(huart, (uint8_t *) IMURvBuff, IMU_DATASIZE);


}
static float last_angle;
static int32_t rotate_times;

float IMU_AngleIncreLoop(float angle_now)
{
    float this_angle;
    this_angle = angle_now;
    if ((this_angle - last_angle) > 300)
        rotate_times--;
    if ((this_angle - last_angle) < -300)
        rotate_times++;
    angle_now = this_angle + rotate_times * 360.0f;
    last_angle = this_angle;
    return angle_now;
}
float Bno085_IMU_Angle(uint8_t which)
{
    Yaw_Angle=IMU_AngleIncreLoop(angle[0]);
    Pih_Angle=angle[1];
    Roll_Angle=angle[2];
    switch (which)
    {
        case 0:
            return -Yaw_Angle;
        case 1:
            return Pih_Angle;
        case 2:
            return Roll_Angle;
    }
    return 0;
}
float Bno085_IMU_Speed(uint8_t which){
    switch (which)
    {
        case 0:
            return Yaw_Speed;
        case 1:
            return Pih_Speed;
        case 2:
            return Roll_Speed;
    }
}

void IMU_Receive_Data(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        IMU_Data_Handler(&huart1);

    }
}