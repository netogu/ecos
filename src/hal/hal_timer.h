#ifndef __HAL_TIMER_H
#define __HAL_TIMER_H

#include <stdint.h>

typedef struct hal_timer_s hal_timer_t;

hal_timer_t * timer_create(uint32_t period_us, void (*on_timeout_cb)(void));
void timer_start(hal_timer_t *self);
void timer_stop(hal_timer_t *self);


#endif // __HAL_TIMER_H