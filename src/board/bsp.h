#pragma once

#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/hrtim.h"

//------------------------------------------------------    
// GPIOs
//------------------------------------------------------

struct board_gpio {
  gpio_t led_green;
};

extern struct board_gpio gpios;
extern struct hrtim_pwm pwm1;
extern uint8_t g_task_wait_flag;

void board_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);