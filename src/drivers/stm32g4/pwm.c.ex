#include "stm32g4/pwm.h"
#include "stm32g4/hrtim.h"

int pwm_init(struct pwm *pwm, uint32_t *base_timer, uint32_t channel, enum pwm_mod mod, enum pwm_output output, uint32_t frequency) {
    pwm->base_timer = base_timer;
    return 0;
}