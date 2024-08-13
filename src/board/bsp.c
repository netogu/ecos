
#include "board/bsp.h"
#include <stdint.h>

#include "drivers/stm32g4/adc.h"
#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/hrtim.h"
#include "drivers/stm32g4/lpuart.h"
#include "drivers/stm32g4/rcc.h"
#include "drivers/stm32g4/spi.h"
#include "drivers/stm32g4/usbpcd.h"
#include "stm32g474xx.h"
#include "tusb.h"
// #include "microshell.h"

//------------------------------------------------------+
// Board Configuration
//------------------------------------------------------+

static void board_clock_setup(void);
static void board_gpio_setup(void);
static void board_serial_setup(void);
static void board_usb_setup(void);
static void board_pwm_setup(void);
static void board_spi_setup(void);

#define STM32G4_NUKLEO 0

void board_init(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();
  board_spi_setup();
  board_usb_setup();
  board_pwm_setup();
}

//------------------------------------------------------
// Clock Config
//------------------------------------------------------
static void board_clock_setup(void) {

  // HSI = 16MHz
  rcc_clock_config_t clock_170MHz_pll_hsi = {
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

  // HSE = 24MHz
  rcc_clock_config_t clock_170MHz_pll_hse = {
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

  rcc_crs_config_t crs_config = {
      .sync_source = RCC_CRS_SYNC_SOURCE_USB,
      .sync_polarity = RCC_CRS_SYNC_POLARITY_RISING,
      .sync_scale = RCC_CRS_SYNC_DIV1,
      .error_limit_value = 34,
      .hsi48_calibration_value = 32,
  };

#if STM32G4_NUKLEO
  (void)clock_170MHz_pll_hse;
  rcc_clock_init(&clock_170MHz_pll_hsi);
#else
  (void)clock_170MHz_pll_hsi;
  rcc_clock_init(&clock_170MHz_pll_hse);
#endif

  rcc_crs_init(&crs_config);

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
}

//------------------------------------------------------
// GPIO Config
//------------------------------------------------------

struct board_io io = {

    // Status LEDs
    .led_red =
        {
            .port = GPIO_PORT_B,
            .pin = GPIO_PIN_6,
            .mode = GPIO_MODE_OUTPUT,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF0,
        },

#if STM32G4_NUKLEO
    .led_green =
        {
            .port = GPIO_PORT_A,
            .pin = GPIO_PIN_5,
            .mode = GPIO_MODE_OUTPUT,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF0,
        },
#else
    .led_green =
        {
            .port = GPIO_PORT_B,
            .pin = GPIO_PIN_7,
            .mode = GPIO_MODE_OUTPUT,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF0,
        },
#endif

    .led_blue =
        {
            .port = GPIO_PORT_B,
            .pin = GPIO_PIN_8,
            .mode = GPIO_MODE_OUTPUT,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF0,
        },

    .drive_enable =
        {
            .port = GPIO_PORT_C,
            .pin = GPIO_PIN_13,
            .mode = GPIO_MODE_OUTPUT,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF0,
        },

    // HRTIM_CHA1
    .pwm_ah =
        {
            .port = GPIO_PORT_A,
            .pin = GPIO_PIN_8,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF13,
        },

    // HRTIM_CHA2
    .pwm_al =
        {
            .port = GPIO_PORT_A,
            .pin = GPIO_PIN_9,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF13,
        },

    // HRTIM_CHF1
    .pwm_bh =
        {
            .port = GPIO_PORT_C,
            .pin = GPIO_PIN_6,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF13,
        },
    // HRTIM_CHF2
    .pwm_bl =
        {
            .port = GPIO_PORT_C,
            .pin = GPIO_PIN_7,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF13,
        },
    // HRTIM_CHE1
    .pwm_ch =
        {
            .port = GPIO_PORT_C,
            .pin = GPIO_PIN_8,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF3,
        },
    // HRTIM_CHE2
    .pwm_cl =
        {
            .port = GPIO_PORT_C,
            .pin = GPIO_PIN_9,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF3,
        },

    // Test pin on 'pwm_dac2' pin
    .test_pin0 =
        {
            .port = GPIO_PORT_F,
            .pin = GPIO_PIN_9,
            .mode = GPIO_MODE_OUTPUT,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_UP,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF0,
        },

    .adc11_test =
        {
            .port = GPIO_PORT_A,
            .pin = GPIO_PIN_0,
            .mode = GPIO_MODE_ANALOG,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_NONE,
            .speed = GPIO_SPEED_LOW,
            .af = GPIO_AF0,
        },

    .pwm_dac_ocp_th =
        {
            .port = GPIO_PORT_F,
            .pin = GPIO_PIN_2,
            .mode = GPIO_MODE_ALTERNATE,
            .type = GPIO_TYPE_PUSH_PULL,
            .pull = GPIO_PULL_NONE,
            .speed = GPIO_SPEED_HIGH,
            .af = GPIO_AF2,
        },

    // HRTIM1_FLT5
    .flt_ocp_n = {.port = GPIO_PORT_B,
                  .pin = GPIO_PIN_0,
                  .mode = GPIO_MODE_ALTERNATE,
                  .type = GPIO_TYPE_OPEN_DRAIN,
                  .pull = GPIO_PULL_NONE,
                  .speed = GPIO_SPEED_HIGH,
                  .af = GPIO_AF13},
    // SPI3
    .spi3_miso = {.port = GPIO_PORT_B,
                  .pin = GPIO_PIN_4,
                  .mode = GPIO_MODE_ALTERNATE,
                  .type = GPIO_TYPE_PUSH_PULL,
                  .pull = GPIO_PULL_NONE,
                  .speed = GPIO_SPEED_HIGH,
                  .af = GPIO_AF6},

    .spi3_mosi = {.port = GPIO_PORT_B,
                  .pin = GPIO_PIN_5,
                  .mode = GPIO_MODE_ALTERNATE,
                  .type = GPIO_TYPE_PUSH_PULL,
                  .pull = GPIO_PULL_NONE,
                  .speed = GPIO_SPEED_HIGH,
                  .af = GPIO_AF6},

    .spi3_clk = {.port = GPIO_PORT_B,
                 .pin = GPIO_PIN_3,
                 .mode = GPIO_MODE_ALTERNATE,
                 .type = GPIO_TYPE_PUSH_PULL,
                 .pull = GPIO_PULL_NONE,
                 .speed = GPIO_SPEED_HIGH,
                 .af = GPIO_AF6},

    .spi3_menc1_cs = {.port = GPIO_PORT_D,
                      .pin = GPIO_PIN_2,
                      .mode = GPIO_MODE_OUTPUT,
                      .type = GPIO_TYPE_PUSH_PULL,
                      .pull = GPIO_PULL_NONE,
                      .speed = GPIO_SPEED_HIGH,
                      .af = GPIO_AF0},
    // SPI4
    .spi4_miso = {.port = GPIO_PORT_E,
                  .pin = GPIO_PIN_5,
                  .mode = GPIO_MODE_ALTERNATE,
                  .type = GPIO_TYPE_OPEN_DRAIN,
                  .pull = GPIO_PULL_UP,
                  .speed = GPIO_SPEED_HIGH,
                  .af = GPIO_AF5},

    .spi4_mosi = {.port = GPIO_PORT_E,
                  .pin = GPIO_PIN_6,
                  .mode = GPIO_MODE_ALTERNATE,
                  .type = GPIO_TYPE_PUSH_PULL,
                  .pull = GPIO_PULL_NONE,
                  .speed = GPIO_SPEED_HIGH,
                  .af = GPIO_AF5},

    .spi4_clk = {.port = GPIO_PORT_E,
                 .pin = GPIO_PIN_2,
                 .mode = GPIO_MODE_ALTERNATE,
                 .type = GPIO_TYPE_PUSH_PULL,
                 .pull = GPIO_PULL_NONE,
                 .speed = GPIO_SPEED_HIGH,
                 .af = GPIO_AF5},

    .spi4_gd_cs = {.port = GPIO_PORT_D,
                   .pin = GPIO_PIN_8,
                   .mode = GPIO_MODE_OUTPUT,
                   .type = GPIO_TYPE_PUSH_PULL,
                   .pull = GPIO_PULL_NONE,
                   .speed = GPIO_SPEED_HIGH,
                   .af = GPIO_AF0},

    .spi4_ltc_cs = {.port = GPIO_PORT_D,
                    .pin = GPIO_PIN_6,
                    .mode = GPIO_MODE_OUTPUT,
                    .type = GPIO_TYPE_PUSH_PULL,
                    .pull = GPIO_PULL_NONE,
                    .speed = GPIO_SPEED_HIGH,
                    .af = GPIO_AF0},

};

static void board_gpio_setup(void) {

  for (int i = 0; i < sizeof(io) / sizeof(gpio_t); i++) {
    gpio_pin_init((gpio_t *)&io + i);
  }
}

//------------------------------------------------------
// UART Config
//------------------------------------------------------

// Hook to printf
// void _putchar(char character) {
//   lpuart_write((uint8_t *)&character, 1);
// }

int _write(int handle, char *data, int size) {
  int count;
  (void)handle; // unused
  for (count = 0; count < size; count++) {
    lpuart_write((uint8_t *)data, 1);
    data++;
  }
  return count;
}

static void board_serial_setup(void) {

  gpio_t lpuart_tx = {
      .port = GPIO_PORT_A,
      .pin = GPIO_PIN_2,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_LOW,
      .af = GPIO_AF12,
  };

  gpio_t lpuart_rx = {
      .port = GPIO_PORT_A,
      .pin = GPIO_PIN_3,
      .mode = GPIO_MODE_ALTERNATE,
      .type = GPIO_TYPE_PUSH_PULL,
      .pull = GPIO_PULL_NONE,
      .speed = GPIO_SPEED_LOW,
      .af = GPIO_AF12,
  };

  gpio_pin_init(&lpuart_tx);
  gpio_pin_init(&lpuart_rx);

  lpuart_config_t lpuart_config = {
      .clock_source = LPUART_CLOCK_SOURCE_PCLK,
      .clock_prescale = LPUART_CLOCK_PRESCALER_1,
      .baudrate = 115200,
      .mode = LPUART_MODE_RX_TX,
      .word_length = LPUART_DATA_BITS_8,
      .stop_bits = LPUART_STOP_BITS_1,
      .parity = LPUART_PARITY_NONE,
      .flow_control = LPUART_FLOW_CONTROL_NONE,
  };

  lpuart_init(&lpuart_config);
}

//------------------------------------------------------
// USB-CDC Config
//------------------------------------------------------

static void board_usb_setup(void) {

  // USB_DM = PA11, USB_DP = PA12

  usbpcd_init();
}

void _putchar(char character) { tud_cdc_write_char(character); }

void HardFault_Handler(void) { __asm("BKPT #0\n"); }

//------------------------------------------------------+
// PWM Config
//------------------------------------------------------+
struct hrtim_pwm pwma = {
    .timer = HRTIM_TIMER_A,
    .output = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
    .polarity = HRTIM_PWM_POLARITY_NORMAL,
    .freq_hz = 10000,
    .deadtime_ns = 200.0,
};

struct hrtim_pwm pwmb = {
    .timer = HRTIM_TIMER_F,
    .output = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
    .polarity = HRTIM_PWM_POLARITY_NORMAL,
    .freq_hz = 10000,
    .deadtime_ns = 200.0,
};

struct hrtim_pwm pwmc = {
    .timer = HRTIM_TIMER_E,
    .output = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
    .polarity = HRTIM_PWM_POLARITY_NORMAL,
    .freq_hz = 10000,
    .deadtime_ns = 200.0,
};

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

  hrtim_init();
  // Change HRTIM prescaler
  hrtim_pwm_init(&pwma);
  hrtim_pwm_init(&pwmb);
  hrtim_pwm_init(&pwmc);
  // hrtim_pwm_swap_output(&pwma);
  // hrtim_pwm_enable_fault_input(&pwma, 5);

  pwm_dac_init();
  // hrtim_pwm_set_duty(&pwma, 50);
  // hrtim_pwm_set_duty(&pwmb, 50);
  // hrtim_pwm_set_duty(&pwmc, 50);

  // HRTIM1->sMasterRegs.MCR |= (HRTIM_MCR_TACEN | HRTIM_MCR_TFCEN |
  // HRTIM_MCR_TECEN); hrtim_pwm_start(&pwma); hrtim_pwm_start(&pwmb);
  // hrtim_pwm_start(&pwmc);
}

//--------------------------------------------------------------------+
// ADC Config
//--------------------------------------------------------------------+

// static void board_adc_setup(void) {
//
//   // struct adc adc1 = {
//   //   .instance = ADC1_BASE,
//   //   .channel_count = 1,
//   // };
//
//   // struct adc_input adc1_input = {
//   //   .channel = 11,
//   //   .sample_time = ADC_SAMPLETIME_2_5,
//   //   .resolution = ADC_RESOLUTION_12B,
//   //   .alignment = ADC_ALIGNMENT_RIGHT,
//   //   .trigger = ADC_TRIGGER_SOFTWARE,
//   // };
//   //
//   // adc_init(&adc1);
//   // adc_enable(&adc1);
//
//
//   // adc_add_input(&adc1, &adc1_input);
// }

//------------------------------------------------------
// SPI Config
//------------------------------------------------------

struct spi spi3 = {
    .instance = SPI3,
    .data_size = 8,
    .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
    .polarity = 0,
    .phase = 0,
};

struct spi spi4 = {
    .instance = SPI4,
    .data_size = 16,
    .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
    .polarity = 0,
    .phase = 1,
};

static void board_spi_setup(void) {

  gpio_pin_set(&io.spi3_menc1_cs);
  spi_init_master(&spi3);

  gpio_pin_set(&io.spi4_gd_cs);
  gpio_pin_set(&io.spi4_ltc_cs);
  spi_init_master(&spi4);
}

//------------------------------------------------------
// Gate Driver Config DRV8353
//------------------------------------------------------

uint8_t drv835x_spi_transfer(uint16_t data_tx, uint16_t *data_rx) {

  gpio_pin_clear(&io.spi4_gd_cs);
  spi_transfer(&spi4, data_tx, data_rx);
  gpio_pin_set(&io.spi4_gd_cs);

  return 0;
}

uint8_t drv835x_drv_en(uint8_t state) {
  if (state) {
    gpio_pin_set(&io.drive_enable);
  } else {
    gpio_pin_clear(&io.drive_enable);
  }

  return 0;
}

struct drv835x gate_driver = {
    .state = 0,
    .status = 0,
    .vgs_status = 0,
    .drive_enable = drv835x_drv_en,
    .spi_transfer = drv835x_spi_transfer,
};
