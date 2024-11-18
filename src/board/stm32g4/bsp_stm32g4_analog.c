#include "bsp.h"
#include "log.h"
#include "stm32g474xx.h"
#include "stm32g4_adc.h"

static adc_t adc1 = {.regs = ADC1};
static adc_t adc2 = {.regs = ADC2};
static adc_t adc3 = {.regs = ADC3};
static adc_t adc4 = {.regs = ADC4};

#define ADC_SAMPLE_TIME_1_5_CYCLES 0x0   // 1.5 ADC clock cycles
#define ADC_SAMPLE_TIME_2_5_CYCLES 0x1   // 2.5 ADC clock cycles
#define ADC_SAMPLE_TIME_6_5_CYCLES 0x2   // 6.5 ADC clock cycles
#define ADC_SAMPLE_TIME_12_5_CYCLES 0x3  // 12.5 ADC clock cycles
#define ADC_SAMPLE_TIME_24_5_CYCLES 0x4  // 24.5 ADC clock cycles
#define ADC_SAMPLE_TIME_47_5_CYCLES 0x5  // 47.5 ADC clock cycles
#define ADC_SAMPLE_TIME_92_5_CYCLES 0x6  // 92.5 ADC clock cycles
#define ADC_SAMPLE_TIME_247_5_CYCLES 0x7 // 247.5 ADC clock cycles
#define ADC_SAMPLE_TIME_640_5_CYCLES 0x8 // 640.5 ADC clock cycles

//------------------------------------------------------
// ADC Config
//------------------------------------------------------
void board_adc_setup(void) {
  board_t *brd = board_get_handle();

  brd->ai = (struct board_ai_s){
      //--- Adc1 ---
      .vm_fb =
          (adc_input_t){
              .name = "vm_fb",
              .channel = 2,
              .scale = 0.019536,
              .offset = 0.0,
              .units = "V",
          },

      //--- Adc2 ---
      .temp_a =
          (adc_input_t){
              .name = "temp_a",
              .channel = 5,
              .scale = 1.0,
              .offset = 0.0,
              .units = "C",
          },

      //--- Adc3 ---
      .ia_fb =
          (adc_input_t){
              .name = "ia_fb",
              .channel = 5,
              .scale = 1.0 / 62.05,
              .offset = -2047.5 / 62.05,
              .units = "A",
          },

      //--- Adc4 ---
      .ib_fb =
          (adc_input_t){
              .name = "ib_fb",
              .channel = 4,
              .scale = 1.0 / 62.05,
              .offset = -2047.5 / 62.05,
              .units = "A",
          },

  };

  adc_register_input(&adc1, &brd->ai.vm_fb, 'i', ADC_SAMPLE_TIME_6_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_a, 'r', ADC_SAMPLE_TIME_247_5_CYCLES);
  adc_register_input(&adc3, &brd->ai.ia_fb, 'i', ADC_SAMPLE_TIME_6_5_CYCLES);
  adc_register_input(&adc4, &brd->ai.ib_fb, 'i', ADC_SAMPLE_TIME_6_5_CYCLES);

  brd->ai.temp_a.data = (uint32_t *)&ADC2->DR;

  printf(timestamp());
  if (adc_init(&adc1) == 0) {
    LOG_OK("ADC1");
  } else {
    LOG_FAIL("ADC1");
  }

  printf(timestamp());
  if (adc_init(&adc2) == 0) {
    LOG_OK("ADC2");
  } else {
    LOG_FAIL("ADC2");
  }

  printf(timestamp());
  if (adc_init(&adc3) == 0) {
    LOG_OK("ADC3");
  } else {
    LOG_FAIL("ADC3");
  }

  printf(timestamp());
  if (adc_init(&adc4) == 0) {
    LOG_OK("ADC4");
  } else {
    LOG_FAIL("ADC4");
  }

  // adc_init(&adc2, ADC1);

  adc_start_regular_sampling(&adc2);
  adc_start_injected_sampling(&adc1);
}
