//
// Created by JustinWCola on 2023/7/19.
//

#ifndef LRGB_H
#define LRGB_H

#include <cstdint>
#include "main.h"

class cRgb
{
public:
    cRgb(TIM_HandleTypeDef *tim): _rgb_tim(tim){}

    void init();
    void loop();
private:
    TIM_HandleTypeDef *_rgb_tim;

    void set(uint32_t argb);
};

#endif //LRGB_H
