//
// Created by JustinWCola on 2023/7/19.
//

#include "lrgb.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  500
#define RGB_FLOW_COLOR_LENGTH       3

void cRgb::init()
{
    HAL_TIM_Base_Start(_rgb_tim);
    HAL_TIM_PWM_Start(_rgb_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(_rgb_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(_rgb_tim, TIM_CHANNEL_3);
}

void cRgb::loop()
{
    static uint16_t i, timer;
    static double delta_alpha, delta_red, delta_green, delta_blue;
    static double alpha, red, green, blue;
    static uint32_t argb;
    uint32_t rgb_flow_color[RGB_FLOW_COLOR_LENGTH + 1] = {0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFF0000FF};

    if (timer % (RGB_FLOW_COLOR_LENGTH * RGB_FLOW_COLOR_CHANGE_TIME) == 0)
        i = 0;

    if (timer % RGB_FLOW_COLOR_CHANGE_TIME == 0)
    {
        alpha = (rgb_flow_color[i] & 0xFF000000) >> 24;
        red = ((rgb_flow_color[i] & 0x00FF0000) >> 16);
        green = ((rgb_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((rgb_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha = (double) ((rgb_flow_color[i + 1] & 0xFF000000) >> 24) -
                      (double) ((rgb_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (double) ((rgb_flow_color[i + 1] & 0x00FF0000) >> 16) -
                    (double) ((rgb_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (double) ((rgb_flow_color[i + 1] & 0x0000FF00) >> 8) -
                      (double) ((rgb_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (double) ((rgb_flow_color[i + 1] & 0x000000FF) >> 0) -
                     (double) ((rgb_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

        i++;
    }
    alpha += delta_alpha;
    red += delta_red;
    green += delta_green;
    blue += delta_blue;

    argb = ((uint32_t) (alpha)) << 24 | ((uint32_t) (red)) << 16 | ((uint32_t) (green)) << 8 |
           ((uint32_t) (blue)) << 0;
    set(argb);
    timer++;
}

void cRgb::set(uint32_t argb)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (argb & 0xFF000000) >> 24;
    red = ((argb & 0x00FF0000) >> 16) * alpha;
    green = ((argb & 0x0000FF00) >> 8) * alpha;
    blue = ((argb & 0x000000FF) >> 0) * alpha;

            __HAL_TIM_SetCompare(_rgb_tim, TIM_CHANNEL_1, blue);
            __HAL_TIM_SetCompare(_rgb_tim, TIM_CHANNEL_2, green);
            __HAL_TIM_SetCompare(_rgb_tim, TIM_CHANNEL_3, red);
}
