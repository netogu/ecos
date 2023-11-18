#pragma once

#include "hardware/stm32g4/gpio.h"


#define LED_PORT GPIOA
#define LED_PIN 5


//------------------------------------------------------    
// GPIOs
//------------------------------------------------------
static gpio_t gpio_led_green;


void board_clock_setup(void);
void board_gpio_setup(void);
