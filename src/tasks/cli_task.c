#include "bsp.h"
#include "shell.h"
#include "rtos.h"
#include "shell_header.h"
#include "task_list.h"


static TaskHandle_t cli_task_handle;
static StaticTask_t cli_task_tcb;
static StackType_t cli_task_stack[ TASK_STACK_SIZE_CLI ];
static void cli_task(void * parameters);

TaskHandle_t cli_task_init(void) {
    // Create the task

    struct board_descriptor *board = board_get_descriptor();

    #ifdef SHELL_INTERFACE_USB
        cli_usb_init();
    #elif defined(SHELL_INTERFACE_USART3)
        cli_uart_init(&board->usart3);
    #else
    cli_uart_init(&board->lpuart1);
    #endif

    shell_init();

    cli_task_handle = xTaskCreateStatic( cli_task,
                        xstr(TASK_NAME_CLI),
                        TASK_STACK_SIZE_CLI,
                        NULL,
                        TASK_PRIORITY_CLI,
                        cli_task_stack,
                        &cli_task_tcb);

    if (cli_task_handle != NULL) {
        return cli_task_handle;
    } 
    else {
        return NULL;
    }
}

static void cli_task(void * parameters) {
    (void) parameters;


    // Print Shell Header
    for (int i = 0; i < strlen(shell_head); i++) {
        cli_uart_putc(shell_head[i]);
        vTaskDelay(1);
    }


    while (1) {
        shell_update();
        vTaskDelay(TASK_DELAY_CLI);
    }
}


