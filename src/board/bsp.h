/*--------------------------------------------------------------------------
Board Support Package
File   : bsp.h
--------------------------------------------------------------------------*/

#pragma once

#include "tusb.h"
#include "hal.h"


//------------------------------------------------------+
// Board Variant
//------------------------------------------------------+
#define STM32G4_NUKLEO
// #define STM32G4_F50

//------------------------------------------------------+
// Shell Interface Selection
//------------------------------------------------------+
// #define SHELL_INTERFACE_USB
// #define SHELL_INTERFACE_USART3
#define SHELL_INTERFACE_LPUART1

//------------------------------------------------------
// GPIOs
//------------------------------------------------------
const struct board_descriptor {
  struct brd_gpio_s {
    gpio_t led_red;
    gpio_t led_green;
    gpio_t led_blue;
    gpio_t drive_enable;
    gpio_t test_pin0;
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
    gpio_t pwm_ah;
    gpio_t pwm_al;
    gpio_t pwm_bh;
    gpio_t pwm_bl;
    gpio_t pwm_ch;
    gpio_t pwm_cl;
    gpio_t adc_pa0;
    gpio_t adc_pc0;
    gpio_t adc_pc1;
    gpio_t lpuart_tx;
    gpio_t lpuart_rx;
    gpio_t usart3_tx;
    gpio_t usart3_rx;

  } io;

  pwm_3ph_t mcpwm;

  struct brd_analog_in_s {
    adc_t adc1;
    adc_input_t vbus;
    adc_t adc2;
    adc_input_t temp_a;
    adc_input_t temp_b;
    // adc_input_t temp_c;
  } ain;

  struct spi spi3;
  struct spi spi4;
  
  uart_t lpuart1;
  uart_t usart3;

  encoder_t encoder;
};

// extern struct gpio io;
// extern struct hrtim_pwm pwma, pwmb, pwmc;
// extern struct spi spi3, spi4;
// extern struct drv835x gate_driver; 


struct board_descriptor *board_get_descriptor(void);
int board_init(void);
void board_hw_setup(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);
