/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>


#include <stdint.h>
// #include <stdio.h>
#include "external/printf.h"
#include "board/bsp.h"
// #include "board/shell.h"
#include "tusb.h"
// #include "ush_config.h"

// USB Device IRQ Monitor
extern int usb_lp_irq_counter;

/*-----------------------------------------------------------*/
// Queues
/*-----------------------------------------------------------*/
#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof( uint32_t )
static StaticQueue_t led_queue_buffer;
uint8_t led_queue_storage[QUEUE_LENGTH*ITEM_SIZE]; 
static QueueHandle_t led_queue;

/*-----------------------------------------------------------*/
//  Tasks
/*-----------------------------------------------------------*/

void led_task( void * parameters )
{   
  /* Unused parameters. */
  ( void ) parameters;
  uint32_t led_delay = 1000; 

  while (1) {
    if (xQueueReceive(led_queue, (void *)&led_delay, 0) == pdTRUE) {
      // New Blink Delay Received
    }
    gpio_pin_toggle(&gpios.led_green);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}
/*-----------------------------------------------------------*/

void usb_task( void * parameters )
{   
  /* Unused parameters. */
    ( void ) parameters;

  while (1) {
    tud_task();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
/*-----------------------------------------------------------*/



int main_loop_counter = 0;
int main(void) 
{

  board_init();
  // shell_setup();
  tusb_init();

  led_queue = xQueueCreateStatic(QUEUE_LENGTH,
                             ITEM_SIZE,
                             &( led_queue_storage[ 0 ] ),
                             &led_queue_buffer );


  static StaticTask_t led_task_tcb;
  static StackType_t led_task_stack[ configMINIMAL_STACK_SIZE ];
  xTaskCreateStatic( led_task,
                        "LED Task",
                        configMINIMAL_STACK_SIZE,
                        NULL,
                        configMAX_PRIORITIES - 1,
                        &( led_task_stack[ 0 ] ),
                        &( led_task_tcb ) );

  static StaticTask_t usb_task_tcb;
  static StackType_t usb_task_stack[ configMINIMAL_STACK_SIZE ];
  xTaskCreateStatic( usb_task,
                      "LED Task",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 1,
                      &( usb_task_stack[ 0 ] ),
                      &( usb_task_tcb ) );

  /* Start the scheduler. */
  vTaskStartScheduler();



 

  while (1) 
  { 
    // gpio_pin_toggle(&gpios.led_green);
    // delay_ms(250);
    // Should not reach here
  }
}

//--------------------------------------------------------------------+
// USB Mount Callback API (Optional)
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void) {
  // printf("USB mounted\r\n");
  uint32_t led_delay = 100;
  if (xQueueSend(led_queue, (void *)&led_delay, 10) != pdTRUE) {
            // Could not place value in Queue
  }
}

void _init(void) {
  // printf("init\r\n");
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )
{
    /* Check pcTaskName for the name of the offending task,
     * or pxCurrentTCB if pcTaskName has itself been corrupted. */
    ( void ) xTask;
    ( void ) pcTaskName;
}
/*-----------------------------------------------------------*/
