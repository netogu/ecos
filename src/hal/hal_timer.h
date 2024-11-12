#ifndef __HAL_TIMER_H
#define __HAL_TIMER_H

#include <stdint.h>

typedef struct timer_s timer_t;

timer_t * timer_create(uint32_t period_us, void (*on_timeout_cb)(void));
void timer_start(timer_t *self);
void timer_stop(timer_t *self);


#endif // __HAL_TIMER_H