//
// Created by ShiF on 2023/11/30.
//

#include "refee.hpp"
#define REFEREE_RECIRVE_1 0x601

sReferee_Msg refereeMsg;

void RefereeMsg(uint32_t rx_std_id ,uint8_t *rx_buf)
{
    if (rx_std_id == REFEREE_RECIRVE_1)
    {
        uint32_t floatAsInt = (uint32_t)(*(rx_buf) <<24 | *(rx_buf + 1) << 16) | *(rx_buf + 2) << 8 | *(rx_buf + 3);
        refereeMsg.shoot_spd = static_cast<float>(floatAsInt);
     }
}