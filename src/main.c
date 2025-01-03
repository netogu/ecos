/*
.---------------------.
|                     |
|                     |
|        _  ._  _     |
|    /_// ///_ /_/    |
|                     |
|                     |
'---------------------'
*/

#include "rtos.h"
#include "shell.h"
#include "bsp.h"
#include "tasklist.h"


int main(void) {

  board_init();
  bootup_system();

  /* Start the scheduler. */
  vTaskStartScheduler();

  // Should not reach here
  return 0;
}
