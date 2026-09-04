#ifndef __BSP_CRC_H
#define __BSP_CRC_H
#include <cstdint>


uint8_t Get_CRC8(uint8_t init_value, const uint8_t * ptr, uint8_t len);
uint16_t Get_CRC16(const uint8_t * ptr, uint16_t len);


#endif
