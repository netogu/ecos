#pragma once

#include <stm32g4xx.h>
#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/hrtim.h"

//------------------------------------------------------    
// GPIOs
//------------------------------------------------------

struct board_io {
  gpio_t led_red;
  gpio_t led_green;
  gpio_t led_blue;
  gpio_t drive_enable;
  gpio_t pwm_ah;
  gpio_t pwm_al;
  gpio_t pwm_bh;
  gpio_t pwm_bl;
  gpio_t pwm_ch;
  gpio_t pwm_cl;
  gpio_t test_pin0;
  gpio_t test_pin1;
  gpio_t adc11_test;
  gpio_t pwm_dac_ocp_th;
};

extern struct board_io io;
extern struct hrtim_pwm pwma, pwmb, pwmc;
extern uint8_t g_task_wait_flag;

void board_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);