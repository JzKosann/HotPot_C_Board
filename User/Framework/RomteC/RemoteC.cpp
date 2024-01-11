//
// Created by ShiF on 2023/9/12.
//

#include "RemoteC.h"
#include "usart.h"
#include "stdlib.h"
#include "cstring"
#include "pidC.h"

/**
 *  RemoteC
 *  遥控器
 *  串口 USART1
 */


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data
//遥控器控制变量
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         nonehh
  */
void REMOTEC_Init(void)
{
    REMOTEIO_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void REMOTEC_UartIrqHandler(void)
{
//    usart_printf("hh\r\n");
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else    //  DMA越界？-kosann
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
        RC_DataHandle(&rc_ctrl);
    }
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
bool RC_GetNewData = false;//检测键值是否在发送/更新 如何判断是否连接键盘呢？
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;                //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;         //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |                  //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;         //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                               //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                          //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                            //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                            //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                          //!< Mouse Z axis 滚轮
    rc_ctrl->mouse.press_l.Now_State = sbus_buf[12];                                //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r.Now_State = sbus_buf[13];                                //!< Mouse Right Is Press ?
    rc_ctrl->key.value =
            sbus_buf[14] | (sbus_buf[15] << 8);                                     //!< KeyBoard value  W S A D Q E SHIFT CTRL
    rc_ctrl->key.W = (rc_ctrl->key.value & 0x01);
    rc_ctrl->key.S = (rc_ctrl->key.value & 0x02) >> 1;
    rc_ctrl->key.A = (rc_ctrl->key.value & 0x04) >> 2;
    rc_ctrl->key.D = (rc_ctrl->key.value & 0x08) >> 3;
    rc_ctrl->key.SHIFT.Now_State = (rc_ctrl->key.value & 0x10) >> 4;
    rc_ctrl->key.CONTRL.Now_State = (rc_ctrl->key.value & 0x20) >> 5;
    rc_ctrl->key.Q.Now_State = (rc_ctrl->key.value & 0x40) >> 6;
    rc_ctrl->key.E.Now_State = (rc_ctrl->key.value & 0x80) >> 7;
    rc_ctrl->key.R.Now_State = (rc_ctrl->key.value & 0x100) >> 8;
    rc_ctrl->key.F.Now_State = (rc_ctrl->key.value & 0x200) >> 9;
    rc_ctrl->key.G.Now_State = (rc_ctrl->key.value & 0x400) >> 10;
    rc_ctrl->key.Z.Now_State = (rc_ctrl->key.value & 0x800) >> 11;
    rc_ctrl->key.X.Now_State = (rc_ctrl->key.value & 0x1000) >> 12;
    rc_ctrl->key.C.Now_State = (rc_ctrl->key.value & 0x2000) >> 13;
    rc_ctrl->key.V.Now_State = (rc_ctrl->key.value & 0x4000) >> 14;
    rc_ctrl->key.B.Now_State = (rc_ctrl->key.value & 0x8000) >> 15;
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //拨盘

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    RC_GetNewData = true;
}

bool RC_ErrorData()
{
    return RC_GetNewData;
}

void RC_DataHandle(RC_ctrl_t *rc_ctrl)  //抑制零漂
{
    if (abs(rc_ctrl->rc.ch[0]) < 5)rc_ctrl->rc.ch[0] = 0;
    if (abs(rc_ctrl->rc.ch[1]) < 5)rc_ctrl->rc.ch[1] = 0;
    if (abs(rc_ctrl->rc.ch[2]) < 5)rc_ctrl->rc.ch[2] = 0;
    if (abs(rc_ctrl->rc.ch[3]) < 5)rc_ctrl->rc.ch[3] = 0;
    if (abs(rc_ctrl->rc.ch[4]) < 5)rc_ctrl->rc.ch[4] = 0;
    if (abs(rc_ctrl->rc.ch[0]) > 670 || abs(rc_ctrl->rc.ch[3]) > 670)
    {
        memset(rc_ctrl, 0, sizeof(RC_ctrl_t)); //异常值
    }
}

RC_ctrl_t RC_GetDatas(void)
{
    return rc_ctrl;
}

/*
	* @name   portHandle
	* @brief  非连续键值处理 即上一次是0，本次是1，判断为按了一次，用于处理状态切换等不适用于连续用手按的按键
	* @param  port 键值状态结构体
  	* @retval None
*/
void portHandle(Key_State *port)
{
    if (port->Now_State == 1 && port->Last_State == 0) port->Is_Click_Once = 1;
    else port->Is_Click_Once = 0;
    port->Last_State = port->Now_State;
}


