
#include "bsp.h"
#include "log.h"

//------------------------------------------------------+
// PWM Config
//------------------------------------------------------+

void board_pwm_setup(void) {

  board_t *brd = board_get_handle();

  brd->hw.mcpwm =

      (pwm_3ph_t){
          .pwma =
              (pwm_t){
                  .options =
                      {
                          .pwm_timer = PWM_TIMER_HRTIM1,
                          .pwm_channel = PWM_HRTIM_TIM_A,
                          .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
                          .polarity = HRTIM_PWM_POLARITY_NORMAL,
                      },
              },

          .pwmb =
              (pwm_t){
                  .options =
                      {
                          .pwm_timer = PWM_TIMER_HRTIM1,
                          .pwm_channel = PWM_HRTIM_TIM_F,
                          .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
                          .polarity = HRTIM_PWM_POLARITY_NORMAL,
                      },
              },

          .pwmc =
              (pwm_t){
                  .options =
                      {
                          .pwm_timer = PWM_TIMER_HRTIM1,
                          .pwm_channel = PWM_HRTIM_TIM_E,
                          .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
                          .polarity = HRTIM_PWM_POLARITY_NORMAL,
                      },
              },

          .mode = PWM_3PHASE_MODE_6PWM,
      },

  printf(timestamp());
  if (pwm_3ph_init(&brd->hw.mcpwm, 50000, 200) != 0) {
    LOG_FAIL("PWM");
  } else {
    LOG_OK("PWM");
  }

  pwm_3ph_set_duty(&brd->hw.mcpwm, 0.5f, 0.5f, 0.5f);
  // pwm_enable_adc_trigger_1_on_rst(brd.hw.mcpwm.pwma);
  pwm_3ph_start(&brd->hw.mcpwm);
}

//------------------------------------------------------
// SPI Config
//------------------------------------------------------

__attribute__((unused)) static void board_spi_setup(void) {

  // brd.spi3 =
  //     (struct spi){
  //         .instance = SPI3,
  //         .data_size = 8,
  //         .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
  //         .polarity = 0,
  //         .phase = 1,
  //     },
  //
  // brd.spi4 =
  //     (struct spi){
  //         .instance = SPI4,
  //         .data_size = 16,
  //         .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
  //         .polarity = 0,
  //         .phase = 1,
  //     },
  //
  // gpio_pin_set(&brd.io.spi3_menc1_cs);
  // spi_init_master(&brd.spi3);
  //
  // gpio_pin_set(&brd.io.spi4_gd_cs);
  // gpio_pin_set(&brd.io.spi4_ltc_cs);
  // spi_init_master(&brd.spi4);
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

__attribute__((unused)) static void board_gate_driver_setup(void) {
  // brd.gate_driver.io = &drv835x_io;
}

//------------------------------------------------------
// Encoder
//------------------------------------------------------

void board_encoder_setup(void) {
  board_t *brd = board_get_handle();
  encoder_init(&brd->hw.encoder);
}
