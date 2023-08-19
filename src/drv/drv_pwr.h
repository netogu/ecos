/* Power and Voltage Reference Control*/
#pragma once

#include "drv_common.h"
#include "stm32g4xx.h"
#include <stdint.h>

enum pwr_vos_ranges {
  PWR_VOS_RANGE_X,
  PWR_VOS_RANGE_1,
  PWR_VOS_RANGE_2,
  PWR_VOS_RANGE_XX,
};

void pwr_enable_boost(void);
void pwr_disable_boost(void);
void pwr_set_vos_range(enum pwr_vos_ranges range);
