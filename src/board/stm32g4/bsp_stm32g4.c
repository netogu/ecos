
#include <stdint.h>

#include "bsp.h"
#include "log.h"
#include "rtos.h"
#include "shell.h"

#include "tiny_printf.h"
#include "tusb.h"

extern uint32_t g_isr_count_uart_tx;
extern uint32_t g_isr_count_uart_rx;

static board_t brd;

//------------------------------------------------------+
// Error Handling
//------------------------------------------------------+
void HardFault_Handler(void) { __asm("BKPT #0\n"); }

#define CRITICAL_ERROR HardFault_Handler()

//------------------------------------------------------+
// Clock Selection
//------------------------------------------------------+
#ifdef STM32G4_NUKLEO
#define CLOCK_SETUP_HSI_16MHZ_170MHZ
#else
#define CLOCK_SETUP_HSE_24MHZ_170MHZ
#endif

//------------------------------------------------------+
// System Boot
//------------------------------------------------------+

board_t *board_get_handle(void) { return &brd; }

static void board_clock_setup(void);
static void board_dio_setup(void);
static void board_com_setup(void);
extern void board_encoder_setup(void);
extern void board_adc_setup(void);
extern void board_pwm_setup(void);
extern void board_spi_setup(void);
extern void board_gate_driver_setup(void);
extern void board_usb_setup(void);

int board_init(void) {

  if (0 != board_load_pinmap(&brd)) {
    CRITICAL_ERROR;
  }

  board_clock_setup();
  board_dio_setup();
  board_com_setup();

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

static void board_dio_setup() {

  for (size_t i = 0; i < sizeof(brd.dio) / sizeof(gpio_t); i++) {
    gpio_pin_init((gpio_t *)&brd.dio + i);
  }
}

//------------------------------------------------------
// Comms Setup
//------------------------------------------------------

static void board_com_setup(void) {

  brd.com.console =

      (uart_t){
          .instance = LPUART1,
          .config =
              (struct uart_config_s){
                  .baudrate = 115200,
                  .mode = UART_MODE_RX_TX,
                  .word_length = UART_DATA_BITS_8,
                  .stop_bits = UART_STOP_BITS_1,
                  .parity = UART_PARITY_NONE,
                  .flow_control = UART_FLOW_CONTROL_NONE,
              },

      };

  uart_init_dma(&brd.com.console);
}
