/* Power and Voltage Reference Control*/

#ifndef _PWR_H
#define _PWR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"

typedef enum PWR_VOS_RANGE {
    PWR_VOS_RANGE_1 = PWR_CR1_VOS_0,
    PWR_VOS_RANGE_2 = PWR_CR1_VOS_1,
} pwr_VosRangeType;

void pwr_enable_boost(void);
void pwr_disable_boost(void);
void pwr_set_vos_range(pwr_VosRangeType range);



#endif //_PWR_H