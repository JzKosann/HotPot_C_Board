//
// Created by ShiF on 2023/9/16.
#include "core.h"
#include "gimbalc.hpp"
#include "main.h"

void UserInit()
{
    can_filter_init(&hcan1);
    can_filter_init(&hcan2);
    DEBUGC_UartInit();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    IMU_UartInit();
    delay_init();

    REMOTEC_Init();
    Gimbal_Init();
}

int main()
{
    main_Init();
    UserInit();
//    MX_IWDG_Init();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    OS_Init();

    while (1)
    {
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM4)  //8ms
    {
        CarCanMxg();
    }
    /* USER CODE END Callback 1 */
}


