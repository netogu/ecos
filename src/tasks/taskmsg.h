#ifndef __TASKMSG_H
#define __TASKMSG_H

#include "rtos.h"
#include "shell.h"
#include <stdbool.h>

typedef struct pwmcon_msg_t {
    uint32_t vq_mv;
    uint32_t vd_mv;
    uint32_t vbus_mv;
    uint32_t count_rate;
    bool     manual;
} pwmcon_msg_t;

// PWM Control Queues
extern QueueHandle_t task_pwmcon_queue;

void task_msg_init(void);
void task_pwmcon_msg_send(pwmcon_msg_t msg);
bool task_pwmcon_msg_receive(pwmcon_msg_t *msg);


#endif // __TASKMSG_H