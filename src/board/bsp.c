
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

    rcc_crs_config_t crs_config = {
        .sync_source = RCC_CRS_SYNC_SOURCE_USB,
        .sync_polarity = RCC_CRS_SYNC_POLARITY_RISING,
        .sync_scale = RCC_CRS_SYNC_DIV1,
        .error_limit_value = 34,
        .hsi48_calibration_value = 32,
    };

  rcc_clock_init(&clock_170MHz_pll_hsi);
  rcc_crs_init(&crs_config);
  
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);

}

//------------------------------------------------------
// GPIO Config
//------------------------------------------------------

struct board_io io = {
  .led_green = {.port = GPIO_PORT_A,
                .pin = GPIO_PIN_5,
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
                .af = GPIO_AF0,}};



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
struct hrtim_pwm pwm1 = {
  .timer = HRTIM_TIMER_A,
  .type = HRTIM_PWM_TYPE_CENTER_ALIGNED,
  .output = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
  .polarity = HRTIM_PWM_POLARITY_NORMAL,
  .freq_hz = 100000, 
  .deadtime_ns = 100.0,
};

static void board_pwm_setup(void) {

  hrtim_init();
  hrtim_pwm_init(&pwm1);
  hrtim_pwm_set_duty(&pwm1, 32.5);
  hrtim_pwm_set_n_cycle_run(&pwm1, 3);
  // Enable Reset Interrupt
  HRTIM1->sTimerxRegs[HRTIM_TIM_A].TIMxDIER |= HRTIM_TIMDIER_RSTIE;
  // Enable ADC1 Trigger on Timer A Reset

  hrtim_pwm_start(&pwm1);
}

//--------------------------------------------------------------------+
// ADC Config
//--------------------------------------------------------------------+


static void board_adc_setup(void) {

  struct adc adc1 = {
    .instance = ADC1_BASE,
    .channel_count = 1,
  };

  adc_init(&adc1);
  adc_enable(&adc1);

  // struct adc_input adc1_input = {
  //   .channel = 11,
  //   .sample_time = ADC_SAMPLETIME_2_5,
  //   .resolution = ADC_RESOLUTION_12B,
  //   .alignment = ADC_ALIGNMENT_RIGHT,
  //   .trigger = ADC_TRIGGER_SOFTWARE,
  // };

  adc_add_input(&adc1, &adc1_input);
  adc_start(&adc1);
  adc_stop(&adc1);
  adc_deinit(&adc1);
}
