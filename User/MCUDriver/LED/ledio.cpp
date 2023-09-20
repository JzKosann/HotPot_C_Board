//
// Created by ShiF on 2023/9/14.
//

#include "ledio.h"
#include "ledc.h"
#include "stm32f4xx_hal_gpio.h"
void LEDIO_PortSetLedLevel(LedC led, uint8_t level)
{
    switch (level) {
        case 0:
            HAL_GPIO_WritePin(led.led_port, led.led_pin, GPIO_PIN_SET);
            break;
        case 1:
            HAL_GPIO_WritePin(led.led_port, led.led_pin, GPIO_PIN_RESET);
            break;
        default:
            HAL_GPIO_TogglePin(led.led_port, led.led_pin);
    }
}
/**
  * @brief  用来连接MUC驱动函数和ledc中的控制函数
  * @retval None
  */
//必须先配置，否则调用该函数指针的时候会有问题
void LEDIO_ConfigInit()
{
    setLedLevel = LEDIO_PortSetLedLevel;
}