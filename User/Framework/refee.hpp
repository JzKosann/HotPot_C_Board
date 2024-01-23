//
// Created by ShiF on 2023/11/30.
//

#ifndef KOSANN_UAVGIMBAL_REFEE_HPP
#define KOSANN_UAVGIMBAL_REFEE_HPP
#include "framework_headfile.hpp"


typedef struct
{
    float shoot_spd;
}sReferee_Msg;

extern sReferee_Msg refereeMsg;
void RefereeMsg(uint32_t rx_std_id ,uint8_t *rx_buf);

#endif //KOSANN_UAVGIMBAL_REFEE_HPP
