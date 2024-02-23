#pragma once

#include <stdint.h>


#define HRTIM_PERCLK_HZ 170000000L 

#define HRTIM_FREQ_2_PER(__FREQ__) ((HRTIM_PERCLK_HZ / __FREQ__) * 32 + 1)
#define HRTIM_DT_COUNT_PER_NS(__DT_NS__) (__DT_NS__ / 0.73 + 0.5) // From RM0440 Table 221


#define HRTIM_TIM_A 0
#define HRTIM_TIM_B 1
#define HRTIM_TIM_C 2
#define HRTIM_TIM_D 3
#define HRTIM_TIM_E 4


enum hrtim_timer {
    HRTIM_TIMER_A,
    HRTIM_TIMER_B,
    HRTIM_TIMER_C,
    HRTIM_TIMER_D,
    HRTIM_TIMER_E
};

enum hrtim_pwm_type {
    HRTIM_PWM_TYPE_TRAILING_EDGE,
    HRTIM_PWM_TYPE_CENTER_ALIGNED
};

enum hrtim_pwm_output {
    HRTIM_PWM_OUTPUT_SINGLE,
    HRTIM_PWM_OUTPUT_COMPLEMENTARY,
};

enum hrtim_pwm_polarity {
    HRTIM_PWM_POLARITY_NORMAL,
    HRTIM_PWM_POLARITY_INVERTED
};


struct hrtim_pwm {
    enum hrtim_timer timer;
    enum hrtim_pwm_type type;
    enum hrtim_pwm_output output;
    enum hrtim_pwm_polarity polarity;
    uint32_t freq_hz;
    float deadtime_ns;
};

void hrtim_init(void);
int hrtim_pwm_init(struct hrtim_pwm *pwm);
void hrtim_pwm_set_frequency(struct hrtim_pwm *pwm, uint32_t freq_hz);
void hrtim_pwm_set_duty(struct hrtim_pwm *pwm, uint32_t duty_pc);
void hrtim_pwm_start(struct hrtim_pwm *pwm);
void hrtim_pwm_stop(struct hrtim_pwm *pwm);
void hrtim_pwm_set_n_cycle_run(struct hrtim_pwm *pwm, uint32_t cycles);
// void hrtim_pwm_set_adc_trigger(struct hrtim_pwm *pwm, uint32_t adc_trig);

