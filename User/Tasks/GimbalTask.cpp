//
// Created by ShiF on 2023/9/16.
//

#include "GimbalTask.h"
#include "gimbalc.hpp"
#include "lrgb.h"

void GimbalTask(void const *argument)
{
    /* USER CODE BEGIN GimbalControlTask */
    TickType_t current;
    cRgb RGB(&htim5);
    RGB.init();
    /* Infinite loop */
    for (;;)
    {
//        FeedDog();
        current = xTaskGetTickCount();
        RGB.loop();
        GimbalLoop();

        vTaskDelayUntil(&current, 2/ portTICK_RATE_MS);
    }
    /* USER CODE END GimbalControlTask */
}
