
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

static struct board_descriptor brd;

struct board_descriptor *board_get_descriptor(void) {
  return &brd;
}

int board_init(void) {

  brd = (struct board_descriptor) {

    //------------------------------------------------------+
    // GPIO
    //------------------------------------------------------+

    .io = (struct brd_gpio_s) 
    { 
      .led_red = (gpio_t) {   
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_6, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0,},

    #ifdef STM32G4_NUKLEO
    .led_green = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_5,
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0,},
    #else
      .led_green = (gpio_t) { 
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_7, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0,},
    #endif

      .led_blue = (gpio_t) {  
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_8, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0,},
    
      .drive_enable = (gpio_t) { 
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_13, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0,},

      .test_pin0 = (gpio_t) { 
        .port = GPIO_PORT_F,
        .pin = GPIO_PIN_9, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0,}, 

      .pwm_dac_ocp_th = (gpio_t) { 
        .port = GPIO_PORT_F,
        .pin = GPIO_PIN_2, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF2,},

      .flt_ocp_n = (gpio_t) { 
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_0, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_OPEN_DRAIN,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF13},

      .spi3_miso = (gpio_t) { 
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_4, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF6},

      .spi3_mosi = (gpio_t) { 
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_5, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF6},  

      .spi3_clk = (gpio_t) { 
        .port = GPIO_PORT_B,
        .pin = GPIO_PIN_3, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF6},    

      .spi3_menc1_cs = (gpio_t) { 
        .port = GPIO_PORT_D,
        .pin = GPIO_PIN_2, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0},

      .spi4_miso = (gpio_t) { 
        .port = GPIO_PORT_E,  
        .pin = GPIO_PIN_5, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_OPEN_DRAIN,
        .pull = GPIO_PULL_UP,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF5},

      .spi4_mosi = (gpio_t) { 
        .port = GPIO_PORT_E,  
        .pin = GPIO_PIN_6, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF5},

      .spi4_clk = (gpio_t) { 
        .port = GPIO_PORT_E,
        .pin = GPIO_PIN_2, 
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF5},
      
      .spi4_gd_cs = (gpio_t) { 
        .port = GPIO_PORT_D,
        .pin = GPIO_PIN_8, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0},

      .spi4_ltc_cs = (gpio_t) { 
        .port = GPIO_PORT_D,
        .pin = GPIO_PIN_6, 
        .mode = GPIO_MODE_OUTPUT,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF0},

      .pwm_ah = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_8,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF13,
      },

      .pwm_al = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_9,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF13,
      },

      .pwm_bh = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_6,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF13,
      },
      .pwm_bl = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_7,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF13,
      },
        
      .pwm_ch = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_8,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF3,
      },

      .pwm_cl = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_9,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_HIGH,
        .af = GPIO_AF3,
      },

      .lpuart_tx = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_2,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF12,
      },

      .lpuart_rx = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_3,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF12,
      },

      .usart3_tx = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_10,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF7,
      },

      .usart3_rx = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_11,
        .mode = GPIO_MODE_ALTERNATE,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF7,
      },

      .adc_pa0 = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_ANALOG,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF0,
      },

      .adc_pc0 = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_ANALOG,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF0,
      },

      .adc_pc1 = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_1,
        .mode = GPIO_MODE_ANALOG,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF0,
      },

    },

    //--------------------------------------------------------------------+
    // PWM
    //--------------------------------------------------------------------+

    .mcpwm = (pwm_3ph_t) {

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

    //--------------------------------------------------------------------+
    // SPI
    //--------------------------------------------------------------------+

    .spi3 = (struct spi) {
      .instance = SPI3,
      .data_size = 8,
      .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
      .polarity = 0,
      .phase = 1,
    },

    .spi4 = (struct spi) {
      .instance = SPI4,
      .data_size = 16,
      .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
      .polarity = 0,
      .phase = 1,
    },

    //--------------------------------------------------------------------+
    // UART
    //--------------------------------------------------------------------+

    .lpuart1 = (uart_t) {
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

    .usart3 = (uart_t) {
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

    //--------------------------------------------------------------------+
    // ADC Inputs
    //--------------------------------------------------------------------+
    .ain = {
      //Adc1
      .vbus = (adc_input_t) {
        .name = "vbus",
        .channel = 1,
        .scale = 1.0,
        .offset = 0.0,
        .units = "V",
      },

      //Adc2
      .temp_a = (adc_input_t) {
        .name = "temp_a",
        .channel = 6, 
        .scale = 1.0,
        .offset = 0.0,
        .units = "C",
      },

      .temp_b = (adc_input_t) {
        .name = "temp_b",
        .channel = ADC_CHANNEL_7, 
        .scale = 1.0,
        .offset = 0.0,
        .units = "C",
      },
    },
  };


  board_clock_setup();
  board_gpio_setup();
  board_uart_setup();
  LOG_CLEAR();
  LOG_OK("Core");

  
  printf("SystemCoreClock: %dMHz\r\n", SystemCoreClock/1000000);
  printf("\r\n");

  return 0;
  //TODO return error aggregation

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

  for (int i = 0; i < sizeof(brd.io) / sizeof(gpio_t); i++) {
    gpio_pin_init((gpio_t *)&brd.io + i);
  }
}

//------------------------------------------------------
// UART Config
//------------------------------------------------------


static void board_uart_setup(void) {
  
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
static void pwm_dac_init(void) {
  // Enable TIM20 APB Clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
  // Enable Auto-Reload
  TIM20->CR1 |= TIM_CR1_ARPE;
  // Set count mode to up-count
  TIM20->CR1 &= ~TIM_CR1_DIR;
  // Set Prescaler
  TIM20->PSC = 0;
  // Set Period
  TIM20->ARR = SystemCoreClock / 50000;
  // Set Duty Cycle to 25%
  TIM20->CCR3 = TIM20->ARR >> 2;
  // Set CH3 output mode to PWM
  TIM20->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  // Preload disable
  TIM20->CCMR2 &= ~TIM_CCMR2_OC3PE;
  // Enable CH3 output
  TIM20->CCER |= TIM_CCER_CC3E;
  // Update registers
  TIM20->EGR |= TIM_EGR_UG;
  // Enable Counter
  TIM20->CR1 |= TIM_CR1_CEN;
  TIM20->BDTR |= TIM_BDTR_MOE;
}

static void board_pwm_setup(void) {


  printf(timestamp());
  if(pwm_3ph_init(&brd.mcpwm, 50000, 200) != 0) {
    LOG_FAIL("PWM");
  } else {
    LOG_OK("PWM");
  }

  pwm_3ph_set_duty(&brd.mcpwm, 0.25, 0.5, 0.75);
  pwm_3ph_start(&brd.mcpwm);

  // pwm_init(&brd.pwma, 90000, 200);
  // pwm_init(&brd.pwmb, 90000, 200);
  // pwm_init(&brd.pwmc, 90000, 200);

  // hrtim_pwm_swap_output(&pwma);
  // hrtim_pwm_enable_fault_input(&pwma, 5);

  // pwm_dac_init();
  // pwm_set_duty(&brd.pwma, 0.25);
  // pwm_set_duty(&brd.pwmb, 0.50);
  // pwm_set_duty(&brd.pwmc, 0.80);
  // pwm_swap_output(&brd.pwma);
  pwm_swap_output(&brd.mcpwm.pwmb);
  // pwm_swap_output(&brd.pwmc);
  

  // HRTIM1->sMasterRegs.MCR |= (HRTIM_MCR_TACEN | HRTIM_MCR_TFCEN | HRTIM_MCR_TECEN);
  // pwm_start(&brd.pwma);
  // pwm_start(&brd.pwmb);
  // pwm_start(&brd.pwmc);

}

//------------------------------------------------------
// SPI Config
//------------------------------------------------------

static void board_spi_setup(void) {

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


static void board_gate_driver_setup(void) {
  // brd.gate_driver.io = &drv835x_io;
}


//------------------------------------------------------
// ADC Config
//------------------------------------------------------
void board_adc_setup(void) {

  brd.ain.adc1.options = (struct adc_options_s) {
    .instance = ADC1,
    .clk_domain = ADC_CLK_DOMAIN_HCLK,
  };

  brd.ain.adc2.options = (struct adc_options_s) {
    .instance = ADC2,
    .clk_domain = ADC_CLK_DOMAIN_HCLK,
  };

  printf(timestamp());
  if (adc_init(&brd.ain.adc1) == 0) {
    LOG_OK("ADC1");
  } else {
    LOG_FAIL("ADC1");
  }

  // adc_init(&adc2, ADC1);

  adc_add_regular_input(&brd.ain.adc1, &brd.ain.vbus, ADC_REG_SEQ_ORDER_1, ADC_SAMPLE_TIME_6_5_CYCLES);
  // adc_configure_regular_input(&adc2, &brd.analog_in.temp_a, 0);
  // adc_configure_regular_input(&adc2, &brd.analog_in.temp_b, 1);

  // adc_configure_sample_mode(&brd.ain.adc1, ADC_SAMPLE_MODE_SINGLE);
  brd.ain.vbus.data = &brd.ain.adc1.options.instance->DR;
  adc_start_regular_sampling(&brd.ain.adc1);

}

//------------------------------------------------------
// Encoder
//------------------------------------------------------

void board_encoder_setup(void) {

  gpio_t encoder_pin_a = (gpio_t) {
    .port = GPIO_PORT_A,
    .pin = GPIO_PIN_0,
    .mode = GPIO_MODE_ALTERNATE,
    .af = GPIO_AF2,
    .type = GPIO_TYPE_PUSH_PULL,
    .speed = GPIO_SPEED_HIGH,
  };

  gpio_t encoder_pin_b = (gpio_t) {
    .port = GPIO_PORT_A,
    .pin = GPIO_PIN_1,
    .mode = GPIO_MODE_ALTERNATE,
    .af = GPIO_AF2,
    .type = GPIO_TYPE_PUSH_PULL,
    .speed = GPIO_SPEED_HIGH,
  };

  gpio_pin_init(&encoder_pin_a);
  gpio_pin_init(&encoder_pin_b);

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





