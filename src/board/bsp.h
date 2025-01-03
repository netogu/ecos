/*--------------------------------------------------------------------------
Board Support Package
File   : bsp.h
--------------------------------------------------------------------------*/

#pragma once

#include "hal.h"
#include "shell.h"

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
typedef struct board_s {

  struct board_dio_s {
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
    gpio_t enc_a_pin;
    gpio_t enc_b_pin;
  } dio;

  struct board_ai_s {
    gpio_t adc1_1;
    adc_input_t vm_fb;
    gpio_t adc2_2;
    adc_input_t va_fb;
    gpio_t adc3_12;
    adc_input_t vb_fb;
    gpio_t adc4_3;
    adc_input_t vc_fb;
    adc_input_t temp_a;
    adc_input_t temp_b;
    adc_input_t ia_fb;
    adc_input_t ib_fb;
  } ai;

  struct board_hw_t {
    pwm_3ph_t mcpwm;
    encoder_t encoder;
  } hw;

  struct board_com_t {
    uart_t console;
  } com;

} board_t;

// extern struct gpio io;
// extern struct hrtim_pwm pwma, pwmb, pwmc;
// extern struct spi spi3, spi4;
// extern struct drv835x gate_driver;

board_t *board_get_handle(void);
int board_init(void);
void board_hw_setup(void);

int board_load_pinmap(board_t *self);

int board_start_bootloader(board_t *self);
