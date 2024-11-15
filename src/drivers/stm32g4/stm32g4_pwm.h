#pragma once
#ifndef STM32G4_PWM_H
#define STM32G4_PWM_H


#include "stm32g4_common.h"
#include "stm32g4_gpio.h"


#define HRTIM_DT_COUNT_PER_NS(__DT_NS__) (__DT_NS__ / 0.73 + 0.5) // From RM0440 Table 221

typedef struct pwm_s {
    struct pwm_options_s {
        enum pwm_timer_e {
            PWM_TIMER_HRTIM1,
        } pwm_timer;
        uint16_t  pwm_channel;
        enum pwm_output_mode_e {
            HRTIM_PWM_OUTPUT_SINGLE,
            HRTIM_PWM_OUTPUT_COMPLEMENTARY,
        } output_mode;
        enum pwm_polarity_e {
            HRTIM_PWM_POLARITY_NORMAL,
            HRTIM_PWM_POLARITY_INVERTED
        } polarity;
    } options;
} pwm_t;

typedef struct pwm_3phase_s {
    pwm_t pwma;
    pwm_t pwmb;
    pwm_t pwmc;
    enum pwm_3phase_mode_e {
        PWM_3PHASE_MODE_3PWM,
        PWM_3PHASE_MODE_6PWM,
    } mode;
} pwm_3ph_t;

// STM32G474 Defines
#define PWM_HRTIM_TIM_A 0
#define PWM_HRTIM_TIM_B 1
#define PWM_HRTIM_TIM_C 2
#define PWM_HRTIM_TIM_D 3
#define PWM_HRTIM_TIM_E 4
#define PWM_HRTIM_TIM_F 5
// factory function that configures an existing pwm_t object

// ------------------------------------------------------
// PWM
// ------------------------------------------------------
int pwm_init(pwm_t *self, uint32_t freq_hz, uint32_t dt_ns);
int pwm_start(pwm_t *self);
int pwm_stop(pwm_t *self);
int pwm_set_frequency(pwm_t *self, uint32_t freq_hz);
int pwm_set_duty(pwm_t *self, float duty_u);
int pwm_set_deadtime(pwm_t *self, uint32_t dt_ns);
int pwm_set_n_cycle_run(pwm_t *self, uint32_t cycles);
int pwm_enable_fault_input(pwm_t *self, uint32_t fault);
int pwm_swap_output(pwm_t *self);
int pwm_enable_adc_trigger(pwm_t *self);
int pwm_enable_interrupt(pwm_t *self);

// ------------------------------------------------------
// 3-Phase PWM
// ------------------------------------------------------
int pwm_3ph_init(pwm_3ph_t *self, uint32_t freq_hz, uint32_t dt_ns);
int pwm_3ph_start(pwm_3ph_t *self);
int pwm_3ph_stop(pwm_3ph_t *self);
int pwm_3ph_set_frequency(pwm_3ph_t *self, uint32_t freq_hz);
int pwm_3ph_set_duty(pwm_3ph_t *self, float d1_u, float d2_u, float d3_u);




#endif // STM32G4_PWM_H