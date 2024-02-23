#pragma once

#include <stm32g4xx.h>
#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/hrtim.h"

//------------------------------------------------------    
// GPIOs
//------------------------------------------------------

struct board_io {
  gpio_t led_green;
  gpio_t test_pin1;
  gpio_t adc11_test;
};

extern struct board_io io;
extern struct hrtim_pwm pwm1;
extern uint8_t g_task_wait_flag;

void board_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);