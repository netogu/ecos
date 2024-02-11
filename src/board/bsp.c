
#include <stdint.h>
#include "board/bsp.h"
#include "drivers/stm32g4/rcc.h"
#include "drivers/stm32g4/gpio.h"
#include "drivers/stm32g4/lpuart.h"
#include "drivers/stm32g4/usbpcd.h"
#include "drivers/stm32g4/hrtim.h"
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

//------------------------------------------------------+
// PWM Config
//------------------------------------------------------+
static void board_pwm_setup(void) {

  struct hrtim_pwm pwm1 = {
    .timer = HRTIM_TIMER_A,
    .type = HRTIM_PWM_TYPE_TRAILING_EDGE,
    .output = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
    .polarity = HRTIM_PWM_POLARITY_NORMAL,
    .freq_hz = 334000, 
    .duty_pc = 76.0, 
    .deadtime_ns = 100.0,
  };

  hrtim_init();
  hrtim_pwm_init(&pwm1);
  hrtim_pwm_start(&pwm1);
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

struct board_gpio gpios = {.led_green = {.port = GPIO_PORT_A,
                                         .pin = GPIO_PIN_5,
                                         .mode = GPIO_MODE_OUTPUT,
                                         .type = GPIO_TYPE_PUSH_PULL,
                                         .pull = GPIO_PULL_UP,
                                         .speed = GPIO_SPEED_HIGH,
                                         .af = GPIO_AF0,}};



static void board_gpio_setup(void) {

  for (int i = 0; i < sizeof(gpios) / sizeof(gpio_t); i++) {
    gpio_pin_init((gpio_t *)&gpios + i);
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
