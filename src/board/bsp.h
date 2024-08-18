/*--------------------------------------------------------------------------
Board Support Package
File   : bsp.h
--------------------------------------------------------------------------*/

#pragma once

#include "tusb.h"
#include "hal.h"

#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/pwm.h"
#include "drivers/stm32g4/spi.h"
#include "drivers/power/drv835x.h"
#include "drivers/stm32g4/uart.h"

//------------------------------------------------------
// GPIOs
//------------------------------------------------------


struct brd_gpio_s {

  // change format to PORT_io_name e.x PA11_led_red
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
  gpio_t adc11_test;
  gpio_t pwm_dac_ocp_th;
  gpio_t flt_ocp_n;
  gpio_t spi3_mosi;
  gpio_t spi3_miso;
  gpio_t spi3_clk;
  gpio_t spi3_menc1_cs;
  gpio_t spi4_mosi;
  gpio_t spi4_miso;
  gpio_t spi4_clk;
  gpio_t spi4_gd_cs;
  gpio_t spi4_ltc_cs;
  // TODO create board struct and add alt func ios here too
};

struct board_descriptor {
  struct brd_gpio_s io;
  pwm_t pwma;
  pwm_t pwmb;
  pwm_t pwmc;
  struct spi spi3;
  struct spi spi4;
  struct drv835x gate_driver;
  uart_t lpuart1;
};

// extern struct gpio io;
// extern struct hrtim_pwm pwma, pwmb, pwmc;
// extern struct spi spi3, spi4;
// extern struct drv835x gate_driver; 


struct board_descriptor *board_get_handler(void);
int board_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);
