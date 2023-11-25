#pragma once

#include "hardware/stm32g4/gpio.h"
#include "hardware/stm32g4/lpuart.h"

//------------------------------------------------------    
// GPIOs
//------------------------------------------------------

struct board_gpio {
  gpio_t led_green;
};

extern struct board_gpio gpios;


void board_clock_setup(void);
void board_gpio_setup(void);
void board_serial_setup(void);