
#include <stdint.h>

#include "bsp.h"
#include "rtos.h"
#include "shell.h"
#include "log.h"

#include "tiny_printf.h"
#include "tusb.h"

extern uint32_t g_isr_count_uart_tx;
extern uint32_t g_isr_count_uart_rx;

void HardFault_Handler(void) { __asm("BKPT #0\n"); }


//------------------------------------------------------+
// Clock Selection
//------------------------------------------------------+
#ifdef STM32G4_NUKLEO
#define CLOCK_SETUP_HSI_16MHZ_170MHZ
#else
#define CLOCK_SETUP_HSE_24MHZ_170MHZ
#endif

//------------------------------------------------------+
// Board Configuration
//------------------------------------------------------+

static void board_clock_setup(void);
static void board_gpio_setup(void);
static void board_adc_setup(void);
static void board_uart_setup(void);
static void board_usb_setup(void);
static void board_pwm_setup(void);
static void board_spi_setup(void);
static void board_gate_driver_setup(void);
static void board_encoder_setup(void);

static board_t brd;

board_t *board_get_handle(void) {
  return &brd;
}

// from bsp_pinmap.c
extern int board_pinmap_set(board_t *brd);

int board_init(void) {

  board_pinmap_set(&brd);

  board_clock_setup();
  board_gpio_setup();
  board_uart_setup();
  LOG_CLEAR();
  printf(timestamp());
  LOG_OK("Core");


  return 0;
}

void board_hw_setup(void) {

  board_usb_setup();
  board_pwm_setup();
  board_adc_setup();
  board_encoder_setup();

}




//------------------------------------------------------
// Clock Config
//------------------------------------------------------
static void board_clock_setup() {

  #ifdef CLOCK_SETUP_HSI_16MHZ_170MHZ
    // HSI = 16MHz
    rcc_clock_config_t clock_config = {
        .sysclk_source = RCC_SYSCLK_SOURCE_PLL,
        .pll_source = RCC_PLL_SOURCE_HSI,
        .usbckl_source = RCC_USBCLK_SOURCE_HSI48,
        .pllm = 4,
        .plln = 85,
        .pllp = 2, 
        .pllq = 2,
        .pllr = 2,
        .hclk_scale = RCC_CLK_DIV1,
        .pclk1_scale = RCC_CLK_DIV1,
        .pclk2_scale = RCC_CLK_DIV1,
        .adc12clk_source = RCC_ADC_CLK_SOURCE_NONE,
        .adc345clk_source = RCC_ADC_CLK_SOURCE_NONE,
        .flash_wait_states = 4,
        .vos_range = 1,
        .boost_mode = 1,
    };
  #endif

  #ifdef CLOCK_SETUP_HSE_24MHZ_170MHZ
    // HSE = 24MHz
    rcc_clock_config_t clock_config = {
        .sysclk_source = RCC_SYSCLK_SOURCE_PLL,
        .pll_source = RCC_PLL_SOURCE_HSE,
        .usbckl_source = RCC_USBCLK_SOURCE_HSI48,
        .pllm = 6,
        .plln = 85,
        .pllp = 2, 
        .pllq = 2,
        .pllr = 2,
        .hclk_scale = RCC_CLK_DIV1,
        .pclk1_scale = RCC_CLK_DIV1,
        .pclk2_scale = RCC_CLK_DIV1,
        .adc12clk_source = RCC_ADC_CLK_SOURCE_NONE,
        .adc345clk_source = RCC_ADC_CLK_SOURCE_NONE,
        .flash_wait_states = 4,
        .vos_range = 1,
        .boost_mode = 1,
    };
  #endif

  rcc_crs_config_t crs_config = {
      .sync_source = RCC_CRS_SYNC_SOURCE_USB,
      .sync_polarity = RCC_CRS_SYNC_POLARITY_RISING,
      .sync_scale = RCC_CRS_SYNC_DIV1,
      .error_limit_value = 34,
      .hsi48_calibration_value = 32,
  };

  rcc_clock_init(&clock_config);
  rcc_crs_init(&crs_config);

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
}

//------------------------------------------------------
// GPIO Config
//------------------------------------------------------


static void board_gpio_setup() {

  for (size_t i = 0; i < sizeof(brd.io) / sizeof(gpio_t); i++) {
    gpio_pin_init((gpio_t *)&brd.io + i);
  }
}

//------------------------------------------------------
// UART Config
//------------------------------------------------------


static void board_uart_setup(void) {

  brd.lpuart1 = (uart_t) {
    .instance = LPUART1,
    .config = {
      .baudrate = 115200,
      .mode = UART_MODE_RX_TX,
      .word_length = UART_DATA_BITS_8,
      .stop_bits = UART_STOP_BITS_1,
      .parity = UART_PARITY_NONE,
      .flow_control = UART_FLOW_CONTROL_NONE,
    },
  },

  brd.usart3 = (uart_t) {
    .instance = USART3,
    .config = {
      .baudrate = 115200,
      .mode = UART_MODE_RX_TX,
      .word_length = UART_DATA_BITS_8,
      .stop_bits = UART_STOP_BITS_1,
      .parity = UART_PARITY_NONE,
      .flow_control = UART_FLOW_CONTROL_NONE,
    },
  },

  
    #ifdef SHELL_INTERFACE_USART3
    uart_init(&brd.usart3);
    #else
    // uart_init(&brd.lpuart1);
    uart_init_dma(&brd.lpuart1);
    #endif
}


//------------------------------------------------------
// USB-CDC Config
//------------------------------------------------------

static void board_usb_setup(void) {

  // USB_DM = PA11, USB_DP = PA12

  printf(timestamp());
  if (usbpcd_init() == 0) {
    LOG_OK("USB");
  } else {
    LOG_FAIL("USB");
  };

}



//------------------------------------------------------+
// PWM Config
//------------------------------------------------------+

static void board_pwm_setup(void) {

  brd.mcpwm = (pwm_3ph_t) {

    .pwma = (pwm_t) {
      .options = {
        .pwm_timer = PWM_TIMER_HRTIM1,
        .pwm_channel = PWM_HRTIM_TIM_A,
        .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
        .polarity = HRTIM_PWM_POLARITY_NORMAL,
      },
    },

    .pwmb = (pwm_t) {
      .options = {
        .pwm_timer = PWM_TIMER_HRTIM1,
        .pwm_channel = PWM_HRTIM_TIM_F,
        .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
        .polarity = HRTIM_PWM_POLARITY_NORMAL,
      },
    },

    .pwmc = (pwm_t) {
      .options = {
        .pwm_timer = PWM_TIMER_HRTIM1,
        .pwm_channel = PWM_HRTIM_TIM_E,
        .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
        .polarity = HRTIM_PWM_POLARITY_NORMAL,
      },
    },

    .mode = PWM_3PHASE_MODE_6PWM,
  },

  printf(timestamp());
  if(pwm_3ph_init(&brd.mcpwm, 50000, 200) != 0) {
    LOG_FAIL("PWM");
  } else {
    LOG_OK("PWM");
  }

  pwm_enable_adc_trigger(&brd.mcpwm.pwma);
  pwm_3ph_set_duty(&brd.mcpwm, 0.5f, 0.5f, 0.5f);
  pwm_enable_interrupt(&brd.mcpwm.pwma);
  pwm_3ph_start(&brd.mcpwm);

}

//------------------------------------------------------
// SPI Config
//------------------------------------------------------

__attribute__((unused))
static void board_spi_setup(void) {

  brd.spi3 = (struct spi) {
    .instance = SPI3,
    .data_size = 8,
    .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
    .polarity = 0,
    .phase = 1,
  },

  brd.spi4 = (struct spi) {
    .instance = SPI4,
    .data_size = 16,
    .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
    .polarity = 0,
    .phase = 1,
  },


  gpio_pin_set(&brd.io.spi3_menc1_cs);
  spi_init_master(&brd.spi3);

  gpio_pin_set(&brd.io.spi4_gd_cs);
  gpio_pin_set(&brd.io.spi4_ltc_cs);
  spi_init_master(&brd.spi4);

}

//------------------------------------------------------
// Gate Driver Config DRV8353
//------------------------------------------------------


// static uint8_t drv835x_spi_transfer(uint16_t data_tx, uint16_t *data_rx) {

//   gpio_pin_clear(&brd.io.spi4_gd_cs);
//   spi_transfer(&brd.spi4, data_tx, data_rx);
//   gpio_pin_set(&brd.io.spi4_gd_cs);

//   return 0;
// }

// static uint8_t drv835x_drv_en(uint8_t state) {
//   if (state) {
//     gpio_pin_set(&brd.io.drive_enable);
//   } else {
//     gpio_pin_clear(&brd.io.drive_enable);
//   }
//   return 0;
// }

// struct drv835x_interface drv835x_io = {
//   .drive_enable = drv835x_drv_en,
//   .spi_transfer = drv835x_spi_transfer,
// };


__attribute__((unused))
static void board_gate_driver_setup(void) {
  // brd.gate_driver.io = &drv835x_io;
}


//------------------------------------------------------
// ADC Config
//------------------------------------------------------
void board_adc_setup(void) {
  brd.ain = (struct board_ain_s) {
    //--- Adc1 ---
    .vm_fb = (adc_input_t) {
      .name = "vm_fb",
      .channel = 2,
      .scale = 0.019536,
      .offset = 0.0,
      .units = "V",
    },

    //--- Adc2 ---
    .temp_a = (adc_input_t) {
      .name = "temp_a",
      .channel = 5, 
      .scale = 1.0,
      .offset = 0.0,
      .units = "C",
    },

    //--- Adc3 ---
    .ia_fb = (adc_input_t) {
      .name = "ia_fb",
      .channel = 5,
      .scale = 1.0/62.05,
      .offset = -2047.5/62.05,
      .units = "A",
    },
    
    //--- Adc4 ---
    .ib_fb = (adc_input_t) {
      .name = "ib_fb",
      .channel = 4,
      .scale = 1.0/62.05,
      .offset = -2047.5/62.05,
      .units = "A",
    },

  },

  brd.ain.adc1.options = (struct adc_options_s) {
    .instance = ADC1,
    .clk_domain = ADC_CLK_DOMAIN_HCLK,
  };

  brd.ain.adc2.options = (struct adc_options_s) {
    .instance = ADC2,
    .clk_domain = ADC_CLK_DOMAIN_HCLK,
  };

  brd.ain.adc3.options = (struct adc_options_s) {
    .instance = ADC3,
    .clk_domain = ADC_CLK_DOMAIN_HCLK,
  };

  brd.ain.adc4.options = (struct adc_options_s) {
    .instance = ADC4,
    .clk_domain = ADC_CLK_DOMAIN_HCLK,
  };

  adc_register_input(&brd.ain.adc1, &brd.ain.vm_fb, ADC_INPUT_TYPE_INJECTED, ADC_SAMPLE_TIME_6_5_CYCLES);
  adc_register_input(&brd.ain.adc2, &brd.ain.temp_a, ADC_INPUT_TYPE_REGULAR, ADC_SAMPLE_TIME_6_5_CYCLES);
  adc_register_input(&brd.ain.adc3, &brd.ain.ia_fb, ADC_INPUT_TYPE_INJECTED, ADC_SAMPLE_TIME_6_5_CYCLES);
  adc_register_input(&brd.ain.adc4, &brd.ain.ib_fb, ADC_INPUT_TYPE_INJECTED, ADC_SAMPLE_TIME_6_5_CYCLES);

  // printf(timestamp());
  // if (adc_init(&brd.ain.adc1) == 0) {
  //   LOG_OK("ADC1");
  // } else {
  //   LOG_FAIL("ADC1");
  // }

  // printf(timestamp());
  // if (adc_init(&brd.ain.adc2) == 0) {
  //   LOG_OK("ADC2");
  // } else {
  //   LOG_FAIL("ADC2");
  // }

  // printf(timestamp());
  // if (adc_init(&brd.ain.adc3) == 0) {
  //   LOG_OK("ADC3");
  // } else {
  //   LOG_FAIL("ADC3");
  // }

  // printf(timestamp());
  // if (adc_init(&brd.ain.adc4) == 0) {
  //   LOG_OK("ADC4");
  // } else {
  //   LOG_FAIL("ADC4");
  // }

  // adc_init(&adc2, ADC1);

  // adc_add_injected_input(&brd.ain.adc1, &brd.ain.vm_fb, ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.vm_fb.data = &brd.ain.adc1.options.instance->JDR1;

  // adc_add_regular_input(&brd.ain.adc2, &brd.ain.temp_a, ADC_REG_SEQ_ORDER_1, ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.temp_a.data = &brd.ain.adc2.options.instance->DR;

  // adc_add_injected_input(&brd.ain.adc3, &brd.ain.ia_fb, ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.ia_fb.data = &brd.ain.adc3.options.instance->JDR1;

  // adc_add_injected_input(&brd.ain.adc4, &brd.ain.ib_fb, ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.ib_fb.data = &brd.ain.adc4.options.instance->JDR1;

   
  adc_start_regular_sampling(&brd.ain.adc2);
  // adc_start_injected_sampling(&brd.ain.adc1);
  // adc_start_injected_sampling(&brd.ain.adc3);
  // adc_start_injected_sampling(&brd.ain.adc4);
  adc_enable_injected_input_soc_trigger(&brd.ain.adc1);
  adc_enable_injected_input_soc_trigger(&brd.ain.adc3);
  adc_enable_injected_input_soc_trigger(&brd.ain.adc4);
  
  // adc_enable(&brd.ain.adc1);
  // adc_enable(&brd.ain.adc2);
  // adc_enable(&brd.ain.adc3);
  // adc_enable(&brd.ain.adc4);

}

//------------------------------------------------------
// Encoder
//------------------------------------------------------

void board_encoder_setup(void) {

  encoder_init(&brd.encoder);
}


//------------------------------------------------------
// Syscalls
//------------------------------------------------------

// printf redirect to UART
void _putchar(char character) {

  #ifdef SHELL_INTERFACE_USB
    tud_cdc_write_char(character);
  #elif defined(SHELL_INTERFACE_USART3)
    uart_write(&brd.usart3, (uint8_t *)&character, 1);
  #else 
    uart_write(&brd.lpuart1, (uint8_t *)&character, 1);
  #endif
}

void _init() {
// printf("Hello World!\n");
}

void _close(int file) {
  (void)file;
}

void _fstat(int file, void *st) {
  (void)file;
  (void)st;
}

void _isatty(int file) {
  (void)file;
}

void _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
}

int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len) {
  (void)file;
  #ifdef SHELL_INTERFACE_USB
    tud_cdc_write(ptr, len);
  #elif defined(SHELL_INTERFACE_USART3)
    uart_write(&brd.usart3, (uint8_t *)ptr, len);
  #else
    uart_write(&brd.lpuart1, (uint8_t *)ptr, len);
  #endif
  return len;
}

int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  return -1;
}

int _getpid() {
  return 1;
}





