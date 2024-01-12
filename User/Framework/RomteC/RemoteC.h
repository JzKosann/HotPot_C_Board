//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_REMOTEC_H
#define KOSANN_UAVGIMBAL_REMOTEC_H

#include <cctype>
#include "remoteio.h"
#include "stm32f4xx_hal.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct {
    uint8_t Last_State: 1;
    uint8_t Now_State: 1;
    uint8_t Is_Click_Once: 1;       //“：1”声明后该变量只有0和1
} Key_State;

typedef __packed struct {
    __packed struct {
        int16_t ch[5];  //无符号类型遥控器通道 0、1、2、3 控制信息 1683~1024~364
                        // ch0->右摇杆横向   ch1->右摇杆纵向  ch2->左摇杆横向  ch3->左摇杆纵向
        char s[2];      //遥控发射机S1、S2开关  1:上 2:下 3:中
                        // s[0]->s2(右边)    s[0]->s1(左边)
    } rc;
    __packed struct {
        int16_t x;      //鼠标值X轴 移动速度 32767 ~ -32768 静止值:0
        int16_t y;      //鼠标值Y轴
        int16_t z;      //鼠标值Z轴
//以下为使用到点按键值的键
        Key_State press_l;  //左键是否按下    0没按下 1按下
        Key_State press_r;  //右键是否按下
    } mouse;
    __packed struct {
        uint16_t value;
        uint8_t W: 1;
        uint8_t S: 1;
        uint8_t A: 1;
        uint8_t D: 1;
//以下为使用到点按键值的键
        Key_State SHIFT;
        Key_State CONTRL;
        Key_State Q;
        Key_State E;
        Key_State R;
        Key_State F;
        Key_State G;
        Key_State Z;
        Key_State X;
        Key_State C;
        Key_State V;
        Key_State B;
    } key;
} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void REMOTEC_Init(void);
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
extern const RC_ctrl_t *get_remote_control_point(void);
void RC_DataHandle(RC_ctrl_t *rc_ctrl);
void portHandle(Key_State* port);
RC_ctrl_t RC_GetDatas(void);
extern RC_ctrl_t rc_ctrl;

typedef struct {
    bool now_state;
    bool is_online;
}Online_detect_t;
extern Online_detect_t RC_GetNewData;
#endif //KOSANN_UAVGIMBAL_REMOTEC_H
