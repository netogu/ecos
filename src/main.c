/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>


#include <stdint.h>
// #include <stdio.h>
#include "tusb.h"
#include "tiny_printf.h"

#include "board/bsp.h"
#include "hardware/stm32g4/lpuart.h"
// #include "board/shell.h"
// #include "ush_config.h"

/* Blink pattern
* - 250 ms  : device not mounted
* - 1000 ms : device mounted
* - 2500 ms : device is suspended
*/
enum {
 BLINK_NOT_MOUNTED = 1000,
 BLINK_MOUNTED = 200,
 BLINK_SUSPENDED = 2500,
};


/*-----------------------------------------------------------*/
// Timers
/*-----------------------------------------------------------*/
TimerHandle_t led_blink_timer;
StaticTimer_t led_blink_timer_s;
static void led_blink_cb(TimerHandle_t *xTimer);

/*-----------------------------------------------------------*/
// Queues
/*-----------------------------------------------------------*/

QueueHandle_t serial_queue;
#define SERIAL_QUEUE_LENGTH 50
#define SERIAL_QUEUE_TYPE char
StaticQueue_t serial_queue_s;
uint8_t serial_queue_storage[SERIAL_QUEUE_LENGTH*sizeof(SERIAL_QUEUE_TYPE)];


/*-----------------------------------------------------------*/
//  Tasks
/*-----------------------------------------------------------*/

#define USB_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
TaskHandle_t usb_task_handle;
void UsbTask( void * parameters );

// TaskHandle_t cdc_task_handle;
// void CdcTask( void * parameters );

TaskHandle_t shell_task_handle;
void ShellTask( void * parameters );

int main(void) 
{

  board_init();
  // shell_setup();

  serial_queue = xQueueCreateStatic(SERIAL_QUEUE_LENGTH,
                                   sizeof(SERIAL_QUEUE_TYPE),
                                   &serial_queue_storage[0],
                                   &serial_queue_s);

  led_blink_timer = xTimerCreateStatic(NULL,
                                     pdMS_TO_TICKS(BLINK_NOT_MOUNTED),
                                     true,
                                     NULL,
                                     led_blink_cb,
                                     &led_blink_timer_s);


  static StaticTask_t usb_task_tcb;
  static StackType_t usb_task_stack[USB_TASK_STACK_SIZE];
  usb_task_handle = xTaskCreateStatic( UsbTask,
                      "USB Task",
                      USB_TASK_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 1,
                      &( usb_task_stack[ 0 ] ),
                      &( usb_task_tcb ) );

  // static StaticTask_t cdc_task_tcb;
  // static StackType_t cdc_task_stack[ configMINIMAL_STACK_SIZE ];
  // cdc_task_handle = xTaskCreateStatic( CdcTask,
  //                     "CDC Task",
  //                     configMINIMAL_STACK_SIZE,
  //                     NULL,
  //                     configMAX_PRIORITIES - 2,
  //                     &( cdc_task_stack[ 0 ] ),
  //                     &( cdc_task_tcb ) );

  static StaticTask_t shell_task_tcb;
  static StackType_t shell_task_stack[ configMINIMAL_STACK_SIZE ];
  shell_task_handle = xTaskCreateStatic( ShellTask,
                      "Shell Task",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 1,
                      &( shell_task_stack[ 0 ] ),
                      &( shell_task_tcb ) );

  xTimerStart(led_blink_timer, 0);

  /* Start the scheduler. */
  vTaskStartScheduler();



 

  while (1) 
  { 
    // Should not reach here
  }
}

//--------------------------------------------------------------------+
// USB Mount Callback API (Optional)
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void) {

  xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
  
}

// Invoked when device is un-mounted
void tud_umount_cb(void) {

  xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
  
}
//
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  if (tud_mounted()) {
    xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
  } else {
    xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
  }
}


void UsbTask( void * parameters )
{   
  /* Unused parameters. */
    ( void ) parameters;

  tusb_init();


  while (1) {
    tud_task();
    // tud_cdc_write_flush();
  }

}


// void CdcTask( void * parameters )
// {   
//   /* Unused parameters. */
//     ( void ) parameters;
//
//   char byte;
//
//
//   while (1) {
//
//     // connected() check for DTR bit
//     // Most but not all terminal client set this when making connection
//     // if ( tud_cdc_connected() )
//     // if (xQueueReceive(serial_queue, (void *)&byte, 1) == pdTRUE) {
//     //   // New CDC message Received
//     //   tud_cdc_write_char(byte);
//     //   tud_cdc_write_flush();
//     //   // lpuart_write((uint8_t *)&byte, 1);
//     // }
//
//     vTaskDelay(1);
//
//
//   }
// }

void ShellTask( void * parameters )
{
  static int counter = 0;
  char message[50];

  while (1) {
    sprintf(message, "Counter = %d\r\n", counter);
    for (int i = 0; message[i] != '\0'; i++) {
      xQueueSend(serial_queue, &message[i],1);
    }
    counter++;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}
/*-----------------------------------------------------------*/


static void led_blink_cb(TimerHandle_t *xHandle)
{   
  /* Unused parameters. */
  ( void ) xHandle;

  gpio_pin_toggle(&gpios.led_green);
}
/*-----------------------------------------------------------*/

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
