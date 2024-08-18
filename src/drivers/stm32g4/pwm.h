#pragma once

#include <stdint.h>
#include "stm32g4/gpio.h"


#define HRTIM_DT_COUNT_PER_NS(__DT_NS__) (__DT_NS__ / 0.73 + 0.5) // From RM0440 Table 221


#define HRTIM_TIM_A 0
#define HRTIM_TIM_B 1
#define HRTIM_TIM_C 2
#define HRTIM_TIM_D 3
#define HRTIM_TIM_E 4
#define HRTIM_TIM_F 5

enum timer_e {
    HRTIM_TIMER_A,
    HRTIM_TIMER_B,
    HRTIM_TIMER_C,
    HRTIM_TIMER_D,
    HRTIM_TIMER_E,
    HRTIM_TIMER_F
};

enum pwm_output_mode_e {
    HRTIM_PWM_OUTPUT_SINGLE,
    HRTIM_PWM_OUTPUT_COMPLEMENTARY,
};

enum pwm_polarity_e {
    HRTIM_PWM_POLARITY_NORMAL,
    HRTIM_PWM_POLARITY_INVERTED
};

enum pwm_timer_e {
    
    PWM_TIMER_A,
    PWM_TIMER_B,
    PWM_TIMER_C,
    PWM_TIMER_D,
    PWM_TIMER_E,
    PWM_TIMER_F

};

enum pwm_timer_id_e {
    PWM_HRTIM1,
};

typedef struct pwm_s {
    gpio_t pin;
    enum pwm_timer_id_e timer_id;
    uint8_t timer_channel;
    struct pwm_options_s {
        enum pwm_output_mode_e output_mode;
        enum pwm_polarity_e polarity;
        uint32_t deadtime;
    } options;

    uint32_t freq_hz;
    float duty_cycle;
    float deadtime_ns;
} pwm_t;

// factory function that configures an existing pwm_t object

int pwm_init(pwm_t *pwm);
int pwm_set_frequency(pwm_t *pwm, uint32_t freq_hz);
int pwm_set_duty(pwm_t *pwm, float duty_pc);
int pwm_start(pwm_t *pwm);
int pwm_stop(pwm_t *pwm);
int pwm_set_n_cycle_run(pwm_t *pwm, uint32_t cycles);
int pwm_enable_fault_input(pwm_t *pwm, uint32_t fault);
int pwm_swap_output(pwm_t *pwm);
// void pwm_set_adc_trigger(pwm_t *pwm, uint32_t adc_trig);

