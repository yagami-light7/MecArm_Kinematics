#ifndef BSP_CRC32_H
#define BSP_CRC32_H
#include "main.h"

extern uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len);
extern bool verify_crc32_check_sum(uint32_t *data, uint32_t len);
extern void append_crc32_check_sum(uint32_t *data, uint32_t len);
#endif
