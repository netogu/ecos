#include "task_list.h"

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
    }
};

//--------------------------------------------------------------------+
// Bootup 
//--------------------------------------------------------------------+

void bootup_system(void) {
    task_manager_init();
}

const size_t task_list_size = sizeof(task_list) / sizeof(task_list[0]);