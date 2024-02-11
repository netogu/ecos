#pragma once

#include <stdint.h>
#include "stm32g4/hrtim.h"

enum pwm_mod {
    PWM_TRAILING_EDGE,
    PWM_LEADING_EDGE,
    PWM_CENTER_ALIGNED,
};

enum pwm_output {
    PWM_SINGLE_OUTPUT,
    PWM_COMPLEMENTARY_OUTPUT,
};

struct pwm {

    uint32_t *base_timer;
    enum pwm_mod mod;
    enum pwm_output output;
};


int pwm_init(struct pwm *pwm, uint32_t *base_timer, uint32_t channel, enum pwm_mod mod, enum pwm_output output, uint32_t frequency);
int pwm_start(struct pwm *pwm);
int pwm_stop(struct pwm *pwm);
int pwm_set_duty_cycle(struct pwm *pwm, uint32_t duty_cycle);
int pwm_set_frequency(struct pwm *pwm, uint32_t frequency);
int dpt_init(struct pwm *pwm, uint32_t *base_timer, uint32_t period, uint32_t channel);