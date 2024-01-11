//
// Created by ShiF on 2023/9/12.
//

#include "IMUC.hpp"
#include <cstring>
#include "usart.h"
#include "DebugC.h"
#include "witreg.hpp"
#include "Wit.h"

/**
 *  IMU读取数据 串口7
 *  WIT智能   型号：WT901C TTL
 */

static float angle[3] = {0};
static float speed[3] = {0};//PID用


static float Yaw_Angle, Yaw_Speed, Pih_Angle, Pih_Speed;
static float last_angle;
static int32_t rotate_times;

uint8_t IMU_RxBuf[IMU_DATASIZE - 1] = {0};

/**
	* @name   IMU_UartInit
	* @brief  IMU串口中断及DMA初始化
	* @param  None
	* @retval None
*/
void WIT_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, IMU_RxBuf, IMU_DATASIZE);
}
/**
 * @name   IMU_AngleIncreLoop
 * @brief  IMU角度连续化处理
 * @param  angle_now 当前IMU反馈角度
 * @retval angle_now 连续化后的IMU反馈角度
 */
static float IMU_AngleIncreLoop(float angle_now)
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

/**
	* @name   IMU_Receive_Data
	* @brief  IMU数据接收及处理
	* @param  None
	* @retval None
*/
void WIT_IMU_Receive_Data(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        IMU_Data_Handler();
        uint8_t rx_len = IMU_DATASIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, IMU_DATASIZE);
        memset(IMU_RxBuf, 0, 32);
        HAL_UART_Receive_DMA(&huart1, IMU_RxBuf, IMU_DATASIZE);
    }
}

/**
	* @name   IMU_Data_Handler
	* @brief  IMU数据处理函数
	* @param  None
	* @retval speed[3],angle[3],quaternion
*/
void IMU_Data_Handler(void)
{
    for (int8_t i = 0; i < 3; i++)
    {
        if (IMU_RxBuf[11 * i] != IMU_HEAD)
        {
            return;
        }
        switch (IMU_RxBuf[i * 11 + 1])
        {
            case IMU_SPEED:
                speed[0] = (short) (((short) IMU_RxBuf[i * 11 + 3] << 8) | IMU_RxBuf[i * 11 + 2]) * 2000.0f / 32768.0f;
                speed[1] = (short) (((short) IMU_RxBuf[i * 11 + 5] << 8) | IMU_RxBuf[i * 11 + 4]) * 2000.0f / 32768.0f;
                speed[2] = (short) (((short) IMU_RxBuf[i * 11 + 7] << 8) | IMU_RxBuf[i * 11 + 6]) * 2000.0f / 32768.0f;
                Pih_Speed = speed[0] / 6.0f;
                Yaw_Speed = speed[2] / 6.0f;
                //usart_printf("%f,%f\r\n",Pih_Speed,Yaw_Speed);
                break;
            case IMU_ANGLE: //√
                angle[0] = (short) (((short) IMU_RxBuf[i * 11 + 3] << 8) | IMU_RxBuf[i * 11 + 2]) * 180.0f / 32768.0f;
                angle[1] = (short) (((short) IMU_RxBuf[i * 11 + 5] << 8) | IMU_RxBuf[i * 11 + 4]) * 180.0f / 32768.0f;
                angle[2] = (short) (((short) IMU_RxBuf[i * 11 + 7] << 8) | IMU_RxBuf[i * 11 + 6]) * 180.0f / 32768.0f;
//                NaiveAngle.roll = angle[1];
//                NaiveAngle.pitch = -angle[0]; //上位机在这里解算有问题
//                NaiveAngle.yaw = angle[2];

                Pih_Angle = angle[0]; //与编码器方向一致
                Yaw_Angle = IMU_AngleIncreLoop(angle[2]);//与编码器方向是一致的，自瞄时出了问题
                break;
            case IMU_QUATERNION:
//                quaternion.w = (short) (((short) IMU_RxBuf[i * 11 + 3] << 8) | IMU_RxBuf[i * 11 + 2]) / 32768.0f;
//                quaternion.x = (short) (((short) IMU_RxBuf[i * 11 + 5] << 8) | IMU_RxBuf[i * 11 + 4]) / 32768.0f;
//                quaternion.y = (short) (((short) IMU_RxBuf[i * 11 + 7] << 8) | IMU_RxBuf[i * 11 + 6]) / 32768.0f;
//                quaternion.z = (short) (((short) IMU_RxBuf[i * 11 + 9] << 8) | IMU_RxBuf[i * 11 + 8]) / 32768.0f;
                break;
        }
    }
}



/**
 * IMU角度返回  (已连续化)
 * @param Witch_angle
 * @return YAW轴 或 PITCH轴
 */
float WIT_IMU_Angle(IMU_data Witch_angle)
{
    switch (Witch_angle)
    {
        case imu_yaw:
            return Yaw_Angle;
        case imu_pitch:
            return Pih_Angle;
        default:
            return 0;
    }
}

/**
 *  IMU角速度返回
 * @param Witch_angle
 * @return  YAW轴 或 PITCH轴
 */
float WIT_IMU_Speed(IMU_data which)
{
    switch (which)
    {
        case imu_yaw:
            return Yaw_Speed;
        case imu_pitch:
            return Pih_Speed;
        default:
            return 0;
    }
}

