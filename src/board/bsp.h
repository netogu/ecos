/*--------------------------------------------------------------------------
Board Support Package
File   : bsp.h
--------------------------------------------------------------------------*/

#pragma once

#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/hrtim.h"
#include "drivers/stm32g4/spi.h"

//------------------------------------------------------
// GPIOs
//------------------------------------------------------

struct board_io {

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

extern struct board_io io;
extern struct hrtim_pwm pwma, pwmb, pwmc;
extern struct spi spi3, spi4;
extern uint8_t g_task_wait_flag;

void board_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);
