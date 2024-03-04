
#include <stdint.h>
#include "board/bsp.h"

#include "drivers/stm32g4/rcc.h"
#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/adc.h"
#include "drivers/stm32g4/lpuart.h"
#include "drivers/stm32g4/usbpcd.h"
#include "drivers/stm32g4/hrtim.h"
#include "drivers/stm32g4/adc.h"
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

#define STM32G4_NUKLEO 1

void board_init(void) {

  board_clock_setup();
  board_gpio_setup();
  board_serial_setup();
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
  rcc_clock_init(&clock_170MHz_pll_hsi);
  #else
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
  .led_red = {.port = GPIO_PORT_B,
                .pin = GPIO_PIN_6,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},

  #if STM32G4_NUKLEO
  .led_green = {.port = GPIO_PORT_A,
                .pin = GPIO_PIN_5,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},
  #else
  .led_green = {.port = GPIO_PORT_B,
                .pin = GPIO_PIN_7,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},
  #endif

  .led_blue = {.port = GPIO_PORT_B,
                .pin = GPIO_PIN_8,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},

  .drive_enable = {.port = GPIO_PORT_C,
                .pin = GPIO_PIN_13,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},

  //HRTIM_CHA1
  .pwm_ah = {.port = GPIO_PORT_A,
                .pin = GPIO_PIN_8,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF13,},

  //HRTIM_CHA2
  .pwm_al = {.port = GPIO_PORT_A,
                .pin = GPIO_PIN_9,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF13,},

  //HRTIM_CHF1
  .pwm_bh = {.port = GPIO_PORT_C,
                .pin = GPIO_PIN_6,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF13,},
  //HRTIM_CHF2
  .pwm_bl = {.port = GPIO_PORT_C,
                .pin = GPIO_PIN_7,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF13,},
  //HRTIM_CHE1
  .pwm_ch = {.port = GPIO_PORT_C,
                .pin = GPIO_PIN_8,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF3,},
  //HRTIM_CHE2
  .pwm_cl = {.port = GPIO_PORT_C,
                .pin = GPIO_PIN_9,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF3,},

  // Test pin on 'pwm_dac2' pin
  .test_pin0 = {.port = GPIO_PORT_F,
                .pin = GPIO_PIN_9,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},

  .test_pin1 = {.port = GPIO_PORT_B,
                .pin = GPIO_PIN_0,
                .mode = GPIO_MODE_OUTPUT,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_UP,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF0,},

  .adc11_test = {.port = GPIO_PORT_A,
                .pin = GPIO_PIN_0,
                .mode = GPIO_MODE_ANALOG,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_NONE,
                .speed = GPIO_SPEED_LOW,
                .af = GPIO_AF0,},

  .pwm_dac_ocp_th = {.port = GPIO_PORT_F,
                .pin = GPIO_PIN_2,
                .mode = GPIO_MODE_ALTERNATE,
                .type = GPIO_TYPE_PUSH_PULL,
                .pull = GPIO_PULL_NONE,
                .speed = GPIO_SPEED_HIGH,
                .af = GPIO_AF2,}};


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
  handle = handle; //unused
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

void _putchar(char character) {
  tud_cdc_write_char(character);
}

void HardFault_Handler(void) {
  __asm("BKPT #0\n");
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_HP_IRQHandler(void) {
  tud_int_handler(0);
}

uint32_t usb_lp_irq_counter = 0;
void USB_LP_IRQHandler(void) {
  tud_int_handler(0);
  usb_lp_irq_counter++;
}

void USBWakeUp_IRQHandler(void) {
  tud_int_handler(0);
}

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
  .freq_hz = 100000, 
  .deadtime_ns = 200.0,
};

struct hrtim_pwm pwmc = {
  .timer = HRTIM_TIMER_E,
  .output = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
  .polarity = HRTIM_PWM_POLARITY_NORMAL,
  .freq_hz = 100000, 
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
  TIM20->ARR = 20;
  // Set Duty Cycle to 50%
  TIM20->CCR3 = 10;
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
  pwm_dac_init();

  hrtim_pwm_set_duty(&pwma, 10);
  hrtim_pwm_set_duty(&pwmb, 0);
  hrtim_pwm_set_duty(&pwmc, 0);
  
 
 
  // hrtim_pwm_set_n_cycle_run(&pwma, 3);50.0); HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxDIER |= HRTIM_TIMDIER_RSTIE;
  // // Enable ADC1 Trigger on Timer A Reset

  // hrtim_pwm_start(&pwma);
  // hrtim_pwm_start(&pwmb);
  // hrtim_pwm_start(&pwmc);
  // HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TF1OEN;
  // HRTIM1->sCommonRegs.OENR |= HRTIM_OENR_TF2OEN;
  // // Start Timer
  // HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TFCEN;
}

//--------------------------------------------------------------------+
// ADC Config
//--------------------------------------------------------------------+


static void board_adc_setup(void) {

  struct adc adc1 = {
    .instance = ADC1_BASE,
    .channel_count = 1,
  };

  // struct adc_input adc1_input = {
  //   .channel = 11,
  //   .sample_time = ADC_SAMPLETIME_2_5,
  //   .resolution = ADC_RESOLUTION_12B,
  //   .alignment = ADC_ALIGNMENT_RIGHT,
  //   .trigger = ADC_TRIGGER_SOFTWARE,
  // };

  adc_init(&adc1);
  adc_enable(&adc1);


  // adc_add_input(&adc1, &adc1_input);
}
