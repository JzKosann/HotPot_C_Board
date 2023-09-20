//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_LEDC_H
#define KOSANN_UAVGIMBAL_LEDC_H

/* Includes ------------------------------------------------------------------*/
#include "cstdint"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
/* Exported defines -----------------------------------------------------------*/
#define LED_Port_IO         GPIO_TypeDef*
#define LED_Port_Pin        uint16_t

/* Exported types ------------------------------------------------------------*/
typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE
} LED_State_e;

/* Exported functions prototypes ---------------------------------------------*/
class LedC {
public:
    LED_State_e state = LED_OFF;
    uint8_t on_level; //点亮电平

    LED_Port_IO led_port;
    LED_Port_Pin led_pin;

    LedC(LED_Port_IO led_Io, LED_Port_Pin led_Pin, uint8_t on_Level = 0) :
            led_port(led_Io), led_pin(led_Pin), on_level(on_Level) {};

    void setStatus(LED_State_e status);

private:
};
/* Exported functions  --------------------------------------------------------*/
// 指向外部一个操作底层的函数
extern void (*setLedLevel)(LedC led, uint8_t level);

void LED_Init(void);

#endif //KOSANN_UAVGIMBAL_LEDC_H
