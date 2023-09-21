//
// Created by ShiF on 2023/9/16.
#include "core.h"
#include "class.hpp"

void User_Init() {
    can_filter_init(&hcan1);
    can_filter_init(&hcan2);
    DEBUGC_UartInit();
    HAL_TIM_Base_Start_IT(&htim5);
    REMOTEC_Init();

    motor_Init();
}

int main(void) {
    main_Init();
    User_Init();
    MX_IWDG_Init();

    OS_Init();

    while (1) {
    }
}