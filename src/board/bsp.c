
#include <stdint.h>

#include "bsp.h"

#include "drivers/stm32g4/adc.h"
#include "drivers/stm32g4/spi.h"
#include "drivers/stm32g4/rcc.h"
#include "drivers/stm32g4/adc.h"
#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/adc.h"
#include "drivers/stm32g4/uart.h"
#include "drivers/stm32g4/usbpcd.h"

#include "tiny_printf.h"

#include "tusb.h"

#include "log.h"

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
static struct board_descriptor brd;

/**
 * @brief Get the board descriptor object 
 * 
 * @return struct board_descriptor* 
 */
struct board_descriptor *board_get_descriptor(void) {
  return &brd;
}

/**
 * @brief Initialize the board
 * 
 * @return Error code 
 */
int board_init(void) {


  board_clock_setup();
  board_gpio_setup();
  board_uart_setup();
  LOG_CLEAR();
  LOG_OK("Core Init");
  // board_spi_setup();
  // board_gate_driver_setup();
  LOG_FAIL("SPI Init");
  board_usb_setup();
  LOG_OK("USB Init");
  board_pwm_setup();
  LOG_OK("PWM Init");
  board_adc_setup();
  LOG_OK("ADC Init");

  LOG_INFO("\r\nBoard Init Complete\r\n");

  return 0;
  //TODO return error aggregation

}


static struct board_descriptor brd = (struct board_descriptor) {

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
  },

  //--------------------------------------------------------------------+
  // PWM
  //--------------------------------------------------------------------+

  .pwma = (pwm_t) {

    .pwmh_pin = (gpio_t) {
      .port = GPIO_PORT_A,
      .pin = GPIO_PIN_8,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_HIGH,
      .af = GPIO_AF13,
    },
    
    .pwml_pin = (gpio_t) {
      .port = GPIO_PORT_A,
      .pin = GPIO_PIN_9,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_HIGH,
      .af = GPIO_AF13,
    },

    .options = {
      .pwm_timer = PWM_TIMER_HRTIM1,
      .pwm_channel = PWM_HRTIM_TIM_A,
      .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
      .polarity = HRTIM_PWM_POLARITY_NORMAL,
    },
  },

  .pwmb = (pwm_t) {

    .pwmh_pin = (gpio_t) {
      .port = GPIO_PORT_C,
      .pin = GPIO_PIN_6,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_HIGH,
      .af = GPIO_AF13,
    },

    .pwml_pin = (gpio_t) {
      .port = GPIO_PORT_C,
      .pin = GPIO_PIN_7,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_HIGH,
      .af = GPIO_AF13,
    },
    
    .options = {
      .pwm_timer = PWM_TIMER_HRTIM1,
      .pwm_channel = PWM_HRTIM_TIM_F,
      .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
      .polarity = HRTIM_PWM_POLARITY_NORMAL,
    },
  },

  .pwmc = (pwm_t) {

    .pwmh_pin = (gpio_t) {
      .port = GPIO_PORT_C,
      .pin = GPIO_PIN_8,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_HIGH,
      .af = GPIO_AF3,
    },

    .pwml_pin = (gpio_t) {
      .port = GPIO_PORT_C,
      .pin = GPIO_PIN_9,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_HIGH,
      .af = GPIO_AF3,
    },

    .options = {
      .pwm_timer = PWM_TIMER_HRTIM1,
      .pwm_channel = PWM_HRTIM_TIM_E,
      .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
      .polarity = HRTIM_PWM_POLARITY_NORMAL,
    },
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
    .tx_pin = (gpio_t) {
      .port = GPIO_PORT_A,
      .pin = GPIO_PIN_2,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_LOW,
      .af = GPIO_AF12,
    },
    .rx_pin = (gpio_t) {
      .port = GPIO_PORT_A,
      .pin = GPIO_PIN_3,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_LOW,
      .af = GPIO_AF12,
    },
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
    .tx_pin = (gpio_t) {
      .port = GPIO_PORT_C,
      .pin = GPIO_PIN_10,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_LOW,
      .af = GPIO_AF7,
    },
    .rx_pin = (gpio_t) {
      .port = GPIO_PORT_C,
      .pin = GPIO_PIN_11,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_LOW,
      .af = GPIO_AF7,
    },
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
  .analog_in = {

    .vbus = (adc_input_t) {
      .name = "vbus",
      .adc_instance = ADC_INSTANCE_1,
      .channel = ADC_CHANNEL_OPAMP_1, 
      .pin = (gpio_t) {
        .port = GPIO_PORT_A,
        .pin = GPIO_PIN_3,
        .mode = GPIO_MODE_ANALOG,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF0,
      },
      .scale = 1.0,
      .offset = 0.0,
      .units = "V",
    },

    .temp_a = (adc_input_t) {
      .name = "temp_a",
      .adc_instance = ADC_INSTANCE_2,
      .channel = ADC_CHANNEL_6, 
      .pin = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_ANALOG,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF0,
      },
      .scale = 1.0,
      .offset = 0.0,
      .units = "C",
    },

    .temp_b = (adc_input_t) {
      .name = "temp_b",
      .adc_instance = ADC_INSTANCE_2,
      .channel = ADC_CHANNEL_7, 
      .pin = (gpio_t) {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_1,
        .mode = GPIO_MODE_ANALOG,
        .type = GPIO_TYPE_PUSH_PULL,
        .pull = GPIO_PULL_NONE,
        .speed = GPIO_SPEED_LOW,
        .af = GPIO_AF0,
      },
      .scale = 1.0,
      .offset = 0.0,
      .units = "C",
    },
  },
};



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
    uart_init(&brd.lpuart1);
    #endif
}


//------------------------------------------------------
// USB-CDC Config
//------------------------------------------------------

static void board_usb_setup(void) {

  // USB_DM = PA11, USB_DP = PA12

  usbpcd_init();
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

  static pwm_3ph_t pwm3ph = {
    .mode = PWM_3PHASE_MODE_6PWM,
    .pwm_h = {&brd.pwma, &brd.pwmb, &brd.pwmc},
  };

  pwm_3ph_init(&pwm3ph, 50000, 200);
  pwm_3ph_set_duty(&pwm3ph, 0.25, 0.5, 0.75);
  pwm_3ph_start(&pwm3ph);

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
  pwm_swap_output(&brd.pwmb);
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


static uint8_t drv835x_spi_transfer(uint16_t data_tx, uint16_t *data_rx) {

  gpio_pin_clear(&brd.io.spi4_gd_cs);
  spi_transfer(&brd.spi4, data_tx, data_rx);
  gpio_pin_set(&brd.io.spi4_gd_cs);

  return 0;
}

static uint8_t drv835x_drv_en(uint8_t state) {
  if (state) {
    gpio_pin_set(&brd.io.drive_enable);
  } else {
    gpio_pin_clear(&brd.io.drive_enable);
  }
  return 0;
}

struct drv835x_interface drv835x_io = {
  .drive_enable = drv835x_drv_en,
  .spi_transfer = drv835x_spi_transfer,
};


static void board_gate_driver_setup(void) {
  brd.gate_driver.io = &drv835x_io;
}


//------------------------------------------------------
// ADC Config
//------------------------------------------------------
void board_adc_setup(void) {

  static adc_t adc1,adc2;

  adc_init(&adc1, ADC1);
  adc_init(&adc2, ADC1);



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





