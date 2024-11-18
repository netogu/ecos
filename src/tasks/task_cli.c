#include "bsp.h"
#include "rtos.h"
#include "shell.h"
#include "shell_header.h"
#include "tasklist.h"

static TaskHandle_t cli_task_handle;
static StaticTask_t cli_task_tcb;
static StackType_t cli_task_stack[TASK_STACK_SIZE_CLI];
static void cli_task(void *parameters);

TaskHandle_t cli_task_init(void) {
  // Create the task

  board_t *brd = board_get_handle();

  cli_uart_init(&brd->com.console);

  shell_init();

  cli_task_handle =
      xTaskCreateStatic(cli_task, xstr(TASK_NAME_CLI), TASK_STACK_SIZE_CLI,
                        NULL, TASK_PRIORITY_CLI, cli_task_stack, &cli_task_tcb);

  if (cli_task_handle != NULL) {
    return cli_task_handle;
  } else {
    return NULL;
  }
}

static void cli_task(void *parameters) {
  (void)parameters;

  vTaskDelay(100);
  // Print Shell Header
  for (size_t i = 0; i < strlen(shell_head); i++) {
    cli_uart_putc(shell_head[i]);
    vTaskDelay(1);
  }

  while (1) {
    shell_update();
    vTaskDelay(TASK_DELAY_CLI);
  }
}
