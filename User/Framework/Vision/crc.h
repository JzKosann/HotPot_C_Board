//
// Created by LENOVO on 2023/3/8.
//

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>
uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

#endif  // RM_SERIAL_DRIVER__CRC_HPP_
