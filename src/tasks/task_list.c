#include "task_list.h"

const task_descriptor_t task_list[] = {
    {
        .name = xstr(TASK_NAME_CLI),
        .init = cli_task_init,
        .startup = true
    },
    {
        .name = xstr(TASK_NAME_USB),
        .init = usb_task_init,
        .startup = false
    },
};

const size_t task_list_size = sizeof(task_list) / sizeof(task_list[0]);