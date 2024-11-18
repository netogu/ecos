#include "bsp.h"

extern board_t brd;

//------------------------------------------------------
// ADC Config
//------------------------------------------------------
void board_adc_setup(void) {
  // brd.ai = (struct board_ai_s){
  //     //--- Adc1 ---
  //     .vm_fb =
  //         (adc_input_t){
  //             .name = "vm_fb",
  //             .channel = 2,
  //             .scale = 0.019536,
  //             .offset = 0.0,
  //             .units = "V",
  //         },
  //
  //     //--- Adc2 ---
  //     .temp_a =
  //         (adc_input_t){
  //             .name = "temp_a",
  //             .channel = 5,
  //             .scale = 1.0,
  //             .offset = 0.0,
  //             .units = "C",
  //         },
  //
  //     //--- Adc3 ---
  //     .ia_fb =
  //         (adc_input_t){
  //             .name = "ia_fb",
  //             .channel = 5,
  //             .scale = 1.0 / 62.05,
  //             .offset = -2047.5 / 62.05,
  //             .units = "A",
  //         },
  //
  //     //--- Adc4 ---
  //     .ib_fb =
  //         (adc_input_t){
  //             .name = "ib_fb",
  //             .channel = 4,
  //             .scale = 1.0 / 62.05,
  //             .offset = -2047.5 / 62.05,
  //             .units = "A",
  //         },
  //
  // };
  //
  // printf(timestamp());
  // if (adc_init(&brd.ain.adc1) == 0) {
  //   LOG_OK("ADC1");
  // } else {
  //   LOG_FAIL("ADC1");
  // }
  //
  // printf(timestamp());
  // if (adc_init(&brd.ain.adc2) == 0) {
  //   LOG_OK("ADC2");
  // } else {
  //   LOG_FAIL("ADC2");
  // }
  //
  // printf(timestamp());
  // if (adc_init(&brd.ain.adc3) == 0) {
  //   LOG_OK("ADC3");
  // } else {
  //   LOG_FAIL("ADC3");
  // }
  //
  // printf(timestamp());
  // if (adc_init(&brd.ain.adc4) == 0) {
  //   LOG_OK("ADC4");
  // } else {
  //   LOG_FAIL("ADC4");
  // }
  //
  // // adc_init(&adc2, ADC1);
  //
  // adc_add_injected_input(&brd.ain.adc1, &brd.ain.vm_fb,
  //                        ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.vm_fb.data = &brd.ain.adc1.options.instance->JDR1;
  //
  // adc_add_regular_input(&brd.ain.adc2, &brd.ain.temp_a, ADC_REG_SEQ_ORDER_1,
  //                       ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.temp_a.data = &brd.ain.adc2.options.instance->DR;
  //
  // adc_add_injected_input(&brd.ain.adc3, &brd.ain.ia_fb,
  //                        ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.ia_fb.data = &brd.ain.adc3.options.instance->JDR1;
  //
  // adc_add_injected_input(&brd.ain.adc4, &brd.ain.ib_fb,
  //                        ADC_SAMPLE_TIME_6_5_CYCLES);
  // brd.ain.ib_fb.data = &brd.ain.adc4.options.instance->JDR2;
  //
  // adc_start_regular_sampling(&brd.ain.adc2);
  // adc_enable_injected_input_soc_trigger(&brd.ain.adc1);
  // adc_enable_injected_input_soc_trigger(&brd.ain.adc3);
  // adc_enable_injected_input_soc_trigger(&brd.ain.adc4);
}
