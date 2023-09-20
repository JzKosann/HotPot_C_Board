//
// Created by ShiF on 2023/9/14.
//

#include "laser.h"
//激光模块
void Laser_On(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
}

void Laser_Off(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
}
