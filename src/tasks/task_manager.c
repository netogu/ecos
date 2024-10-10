
#include "bsp.h"
#include "shell.h"
#include "rtos.h"
#include "hal.h"
#include "tiny_printf.h"
#include "task_list.h"

static TaskHandle_t task_manager_handle;
static StaticTask_t task_manager_tcb;
static StackType_t task_manager_stack[ TASK_STACK_SIZE_TASK_MANAGER ];
static void task_manager(void * parameters);

TaskHandle_t task_manager_init(void) {
    // Create the task

    task_manager_handle = xTaskCreateStatic( task_manager,
                        xstr(TASK_NAME_TASK_MANAGER),
                        TASK_STACK_SIZE_TASK_MANAGER,
                        NULL,
                        TASK_PRIORITY_TASK_MANAGER,
                        task_manager_stack,
                        &task_manager_tcb);

    if (task_manager_handle != NULL) {
        printf(timestamp());
        printf("Task Manager Started\r\n");
        return task_manager_handle;
    } 
    else {
        printf(timestamp());
        printf("Task Manager Failed to Start\r\n");
        return NULL;
    }

}

static void task_manager(void * parameters) {
    (void) parameters;

    vTaskDelay(25);
    // while (cli_uart_tx_pending()){
    //     vTaskDelay(1);
    // }

    // Start all tasks
    for (int i = 0; i < task_list_size; i++) {
        if (task_list[i].startup) {
            TaskHandle_t task_handle = task_list[i].init();
            if ( task_handle != NULL) {
                cli_printf(timestamp());
                cli_printf("Started Task : %s\r\n", task_list[i].name);
            } else {
                cli_printf(timestamp());
                cli_printf("Task '%s' Failed to Start\r\n", task_list[i].name);
            }
        }
    }

    while (1) {
        vTaskDelay(TASK_DELAY_TASK_MANAGER);
    }
}