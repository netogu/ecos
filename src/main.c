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
#include "ush_config.h"

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
static void led_blink_cb(TimerHandle_t xTimer);

/*-----------------------------------------------------------*/
// Queues
/*-----------------------------------------------------------*/

// QueueHandle_t serial_queue;
// #define SERIAL_QUEUE_LENGTH 50
// #define SERIAL_QUEUE_TYPE char
// StaticQueue_t serial_queue_s;
// uint8_t serial_queue_storage[SERIAL_QUEUE_LENGTH*sizeof(SERIAL_QUEUE_TYPE)];


/*-----------------------------------------------------------*/
//  Tasks
/*-----------------------------------------------------------*/

#define USB_TASK_STACK_SIZE 1024
TaskHandle_t usbd_task_handle;
StaticTask_t usbd_task_tcb;
StackType_t usbd_task_stack[ USB_TASK_STACK_SIZE ];
static void usbd_task( void * parameters );

#define CDC_TASK_STACK_SIZE 1024
TaskHandle_t cdc_task_handle;
StaticTask_t cdc_task_tcb;
StackType_t cdc_task_stack[ CDC_TASK_STACK_SIZE ];
void cdc_task( void * parameters );

// #define SHELL_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
// TaskHandle_t shell_task_handle;
// StaticTask_t shell_task_tcb;
// StackType_t shell_task_stack[ configMINIMAL_STACK_SIZE ];
// void shell_task( void * parameters );


int main(void) 
{

  board_init();

  // serial_queue = xQueueCreateStatic(SERIAL_QUEUE_LENGTH,
  //                                  sizeof(SERIAL_QUEUE_TYPE),
  //                                  &serial_queue_storage[0],
  //                                  &serial_queue_s);

  led_blink_timer = xTimerCreateStatic(NULL,
                                     pdMS_TO_TICKS(BLINK_NOT_MOUNTED),
                                     true,
                                     NULL,
                                     led_blink_cb,
                                     &led_blink_timer_s);


  usbd_task_handle = xTaskCreateStatic( usbd_task,
                      "usb_task",
                      USB_TASK_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 1,
                      usbd_task_stack,
                      &usbd_task_tcb);

  cdc_task_handle = xTaskCreateStatic( cdc_task,
                      "cdc_task",
                      CDC_TASK_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 2,
                      cdc_task_stack,
                      &cdc_task_tcb);

  // shell_task_handle = xTaskCreateStatic( shell_task,
  //                     "shell_task",
  //                     configMINIMAL_STACK_SIZE,
  //                     NULL,
  //                     configMAX_PRIORITIES - 1,
  //                     shell_task_stack,
  //                     &shell_task_tcb);

  xTimerStart(led_blink_timer, 0);

  /* Start the scheduler. */
  vTaskStartScheduler();



 

  // Should not reach here
  return 0;
}



void usbd_task( void *parameters )
{   
  /* Unused parameters. */
    ( void ) parameters;

  tusb_init();



  while (1) {
    tud_task();
    tud_cdc_write_flush();
  }

}


//--------------------------------------------------------------------+
// USB CDC Device Callbacks
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

//--------------------------------------------------------------------+
// USB CDC Task
//--------------------------------------------------------------------+

void cdc_task( void *parameters )
{   
  /* Unused parameters. */
    ( void ) parameters;

  char byte;

  shell_init();


  while (1) {

    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if ( tud_cdc_connected() )
    {
      // // There are data available
      // while (tud_cdc_available()) {
      //   uint8_t buf[64];

      //   // read and echo back
      //   uint32_t count = tud_cdc_read(buf, sizeof(buf));
      //   (void) count;

      //   // Echo back
      //   // Note: Skip echo by commenting out write() and write_flush()
      //   // for throughput test e.g
      //   //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
      //   tud_cdc_write(buf, count);
      // }
      shell_update();

      tud_cdc_write_flush();
    }

    vTaskDelay(1);

  }
}


// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if (dtr) {
    // Terminal connected
  } else {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void) itf;
}

//--------------------------------------------------------------------+
// SHELL TASK
//--------------------------------------------------------------------+

// void shell_task( void *parameters )
// {
//   static int counter = 0;
//   char message[50];

//   while (1) {
//     sprintf(message, "Counter = %d\r\n", counter);
//     tud_cdc_write_str(message);
//     tud_cdc_write_flush();

//     // for (int i = 0; message[i] != '\0'; i++) {
//     //   xQueueSend(serial_queue, &message[i],1);
//     // }
//     counter++;
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//   }

// }

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+

static void led_blink_cb(TimerHandle_t xTimer)
{   
  /* Unused parameters. */
  ( void ) xTimer;

  gpio_pin_toggle(&gpios.led_green);
}
/*-----------------------------------------------------------*/

void _init(void) {
  // printf("init\r\n");
}

