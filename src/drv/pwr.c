#include "pwr.h"

void pwr_set_vos_range(pwr_VosRangeType scale) {
    uint32_t reg = PWR->CR1;
    reg &= ~PWR_CR1_VOS;
    PWR->CR1 |= ( reg | scale);

}
void pwr_enable_boost(void) {
    PWR->CR5 &= ~PWR_CR5_R1MODE;
}

void pwr_disable_boost(void) {
    PWR->CR5 |= PWR_CR5_R1MODE;
}

