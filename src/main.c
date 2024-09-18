/*
.-----------------------------.
|                             |
|                             |
|                             |
|             8"""88 8""""8   |
|   eeee eeee 8    8 8        |
|   8    8  8 8    8 8eeeee   |
|   8eee 8e   8    8     88   |
|   88   88   8    8 e   88   |
|   88ee 88e8 8eeee8 8eee88   |
|                             |
|                             |
|                             |
'-----------------------------'
*/

#include "rtos.h"
#include "shell.h"
#include "bsp.h"
#include "task_list.h"


int main(void) {

  board_init();

  // Initialize the Task Manager
  task_manager_init();

  /* Start the scheduler. */
  vTaskStartScheduler();

  // Should not reach here
  return 0;
}
