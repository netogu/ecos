/* Embedded Flash memory*/

#ifndef _FLASH_H
#define _FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"

//Flash Access Control - ACR
void flash_set_latency(uint32_t latency);
uint32_t flash_get_latency(void);
void flash_enable_data_cache(void);
void flash_enable_instruction_cache(void);
void flash_disable_data_cache(void);
void flash_disable_instruction_cache(void);



#endif //_FLASH_H