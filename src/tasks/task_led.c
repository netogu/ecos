
//--------------------------------------------------------------------+
// LED Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "tasklist.h"

static TaskHandle_t led_task_handle;
static StaticTask_t led_task_tcb;
static StackType_t led_task_stack[ TASK_STACK_SIZE_LED ];
static void led_task(void * parameters);

TaskHandle_t led_task_init(void) {
    // Create the task

    led_task_handle = xTaskCreateStatic( led_task,
                        xstr(TASK_NAME_LED),
                        TASK_STACK_SIZE_LED,
                        NULL,
                        TASK_PRIORITY_CLI,
                        led_task_stack,
                        &led_task_tcb);

    if (led_task_handle != NULL) {
        return led_task_handle;
    } 
    else {
        return NULL;
    }
}

static void led_task(void * parameters) {   
  struct board_descriptor *brd = board_get_descriptor();
  /* Unused parameters. */
  ( void ) parameters;


  while(1) {
    gpio_pin_toggle(&brd->io.led_green);
    // gpio_pin_toggle(&brd->io.led_red);
    // gpio_pin_toggle(&brd->io.led_blue);
    gpio_pin_set(&brd->io.led_blue);  
    gpio_pin_set(&brd->io.led_red);

    vTaskDelay(TASK_DELAY_LED);
  }
}