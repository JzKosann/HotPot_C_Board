//
// Created by ShiF on 2023/9/16.
#include "core.h"
#include "gimbalc.hpp"

void UserInit() {
    can_filter_init(&hcan1);
    can_filter_init(&hcan2);
    DEBUGC_UartInit();
    HAL_TIM_Base_Start_IT(&htim5);

    REMOTEC_Init();
    Gimbal_Init();
}

int main() {
    main_Init();
    UserInit();
    MX_IWDG_Init();

    OS_Init();

    while (1) {
    }
}



