#include "flash.h"

void flash_set_latency(uint32_t latency) {
    uint32_t reg = FLASH->ACR;
    reg &= ~FLASH_ACR_LATENCY;

    FLASH->ACR |= ( reg | latency );
}

uint32_t flash_get_latency(void) {
    return FLASH->ACR & FLASH_ACR_LATENCY_Msk;
}

void flash_enable_data_cache(void) {
    FLASH->ACR |= FLASH_ACR_DCEN;
}
void flash_enable_instruction_cache(void) {

    FLASH->ACR |= FLASH_ACR_ICEN;
}
void flash_disable_data_cache(void) {

    FLASH->ACR &= ~FLASH_ACR_ICEN;
}
void flash_disable_instruction_cache(void) {

    FLASH->ACR &= ~FLASH_ACR_ICEN;
}