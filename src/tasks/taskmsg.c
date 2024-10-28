#include "taskmsg.h"


//--------------------------------------------------------------------+
// PWM Control Task Message Queue
//--------------------------------------------------------------------+

QueueHandle_t task_pwmcon_queue;


void task_msg_init(void) {
    task_pwmcon_queue = xQueueCreate(1, sizeof(pwmcon_msg_t));
}

void task_pwmcon_msg_send(pwmcon_msg_t msg) {
    xQueueOverwrite(task_pwmcon_queue, &msg);
}

bool task_pwmcon_msg_receive(pwmcon_msg_t *msg) {

    if (xQueueReceive(task_pwmcon_queue, msg, 0) == pdPASS) {
        return true;
        // Message received successfully
    }
    // Don't block if no message is available
    return false;
}
