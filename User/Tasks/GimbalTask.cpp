//
// Created by ShiF on 2023/9/16.
//

#include "GimbalTask.h"
#include "lrgb.h"
#include "classtest.hpp"


void GimbalControlTask(void const *argument) {
    /* USER CODE BEGIN GimbalControlTask */
    cRgb RGB(&htim5);
    TickType_t current;
    RGB.init();
    /* Infinite loop */
    for (;;) {
        FeedDog();
        current = xTaskGetTickCount();
        RGB.loop();
        yaw_cansend(5000);
        vTaskDelayUntil(&current, pdMS_TO_TICKS(10));
    }
    /* USER CODE END GimbalControlTask */
}
