#include "task_list.h"
#include "hal.h"

//--------------------------------------------------------------------+
// Task List
//--------------------------------------------------------------------+

const task_descriptor_t task_list[] = {
    {
        .name = xstr(TASK_NAME_CLI),
        .init = cli_task_init,
        .startup = true
    },
    {
        .name = xstr(TASK_NAME_USB),
        .init = usb_task_init,
        .startup = true
    },
    {
        .name = xstr(TASK_NAME_LED),
        .init = led_task_init,
        .startup = true
    },
    {
        .name = xstr(TASK_NAME_PWM_CONTROL),
        .init = task_pwm_control_init,
        .startup = true
    }
};

//--------------------------------------------------------------------+
// Bootup 
//--------------------------------------------------------------------+

void bootup_system(void) {
    timer_us_init();
    printf(timestamp());
    printf("Booting System\r\n");
    task_manager_init();
}

const size_t task_list_size = sizeof(task_list) / sizeof(task_list[0]);