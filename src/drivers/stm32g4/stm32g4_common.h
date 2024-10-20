#ifndef STM32G4_COMMON_H
#define STM32G4_COMMON_H

#include <stdint.h>
#include "stm32g4xx.h"

#define Set_register_bit(reg, mask) (reg |= (mask))

#define Clear_register_bit(reg, mask) (reg &= ~(mask))

#define Read_register_bit(reg, mask) (reg & (mask))

#define Read_register_field(reg, field) ((uint32_t)reg & field) >> field##_Pos

#define Modify_register_field(reg, field, value)                               \
  {                                                                            \
    uint32_t r = reg;                                                          \
    r &= ~(field);                                                             \
    r |= ((uint32_t)value << field##_Pos) & field##_Msk;                       \
    reg = r;                                                                   \
  }


#define Limit(x, min, max) (x < min ? min : x > max ? max : x)
#define In_range(x, min, max) (x >= min && x <= max)

#endif