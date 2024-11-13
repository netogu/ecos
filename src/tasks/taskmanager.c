
#include "bsp.h"
#include "shell.h"
#include "rtos.h"
#include "hal.h"
#include "tiny_printf.h"
#include "tasklist.h"
#include "taskmsg.h"

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
    

    board_hw_setup();
    task_msg_init();
    vTaskDelay(50);


    // Start all tasks
    for (size_t i = 0; i < task_list_size; i++) {
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

    system_id_t sys_id;
    system_get_id(&sys_id);

    cli_printf("\r\nSystem Clock: %dMHz\r\n", SystemCoreClock/1000000);
    cli_printf("Device ID: (0x%X)", sys_id.device_id);
    if (sys_id.device_id == 0x469) {
        cli_printf(" STM32G474RE\r\n");
    } else {
        cli_printf(" Unknown\r\n");
    }
    cli_printf("Flash Size: %dKB\r\n", sys_id.flash_size_kb);
    cli_printf("Package Type: %d\r\n", sys_id.package_type);
    cli_printf("UID: 0x%X 0x%X 0x%X\r\n", sys_id.uid[0], sys_id.uid[1], sys_id.uid[2]);


    while (1) {
        vTaskDelay(TASK_DELAY_TASK_MANAGER);
    }
}