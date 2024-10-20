/* Embedded Flash memory*/
#ifndef STM32G4_FLASH_H
#define STM32G4_FLASH_H

#include <stdint.h>
#include "stm32g4xx.h"

// Flash Access Control - ACR
void flash_set_latency(uint32_t latency);
uint32_t flash_get_latency(void);
void flash_enable_data_cache(void);
void flash_enable_instruction_cache(void);
void flash_disable_data_cache(void);
void flash_disable_instruction_cache(void);

#endif // STM32G4_FLASH_H