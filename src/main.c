#include <stddef.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

static volatile uint32_t tick_ms_count;

void sleep_ms(uint32_t ms){

  uint32_t current_ms = tick_ms_count;
  while ((tick_ms_count - current_ms) < ms) {
    continue;
  }
}

static void gpio_setup(void){

  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set(GPIOC,GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);


}

static void systick_setup(){

  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(8999);
  systick_interrupt_enable();
  systick_counter_enable();


}

void sys_tick_handler(void){
  tick_ms_count++;
}

int main(void){
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  gpio_setup();
  systick_setup();

  while(1){
    gpio_toggle(GPIOC,GPIO13);
    sleep_ms(250);
  }

}
