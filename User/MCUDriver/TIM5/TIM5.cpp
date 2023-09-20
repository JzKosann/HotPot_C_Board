//
// Created by ShiF on 2023/9/14.
//

#include "TIM5.h"
#include "bsp_can.h"
#include "debugc.h"
#include "remotec.h"
#include "imuc.h"
#include "gimbalc.h"
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//    if (htim == &htim5)
//    {
////        Gimbal_ControlLoop();
//    }
//}

void TIM5_IT_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim5);
    //HAL_TIM_Base_Start_IT(&htim1);
}