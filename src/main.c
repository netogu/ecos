/* FreeRTOS includes. */
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include "portmacro.h"
#include "FreeRTOSConfig.h"

#include "board/bsp.h"
#include "tusb.h"
// #include "tiny_printf.h"

// #include "stm32g4/lpuart.h"
// #include "board/shell.h"
#include "ush_config.h"
// #include "drivers/drv_usb.h"

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

/* 
api::Status Os::cmdPs(lib::console::ConsoleBase& console, lib::console::Arguments& arguments)
{
  static const size_t kMaxTasks = 50U;
  static etl::array<TaskStatus_t, kMaxTasks> task_status_array;

  size_t task_count = uxTaskGetNumberOfTasks();

  uint32_t total_run_time;
  uint32_t stats_as_percentage;

  console.print("FreeRTOS task list\r\n");
  console.print("==================================================\r\n");
  console.print(
    "|{:<20}|{:>4}|{:>10}|{:>4}|{:>6}|\r\n",
      "Task Name",
      "Prio",
      "Cycles",
      "CPU",
      "Stack"
  );
  console.print("--------------------------------------------------\r\n");

  // Generate raw status information about each task.
  task_count = uxTaskGetSystemState(
    task_status_array.data(),
    task_status_array.size(),
    &total_run_time
  );
*/

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

#define BG_TASK_STACK_SIZE 516
TaskHandle_t bg_task_handle;
StaticTask_t bg_task_tcb;
StackType_t bg_task_stack[BG_TASK_STACK_SIZE];
static void bg_task( void *parameters );

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
                      configMAX_PRIORITIES - 2,
                      usbd_task_stack,
                      &usbd_task_tcb);

  cdc_task_handle = xTaskCreateStatic( cdc_task,
                      "cdc_task",
                      CDC_TASK_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 3,
                      cdc_task_stack,
                      &cdc_task_tcb);

  bg_task_handle = xTaskCreateStatic( bg_task,
                      "bg_task",
                      BG_TASK_STACK_SIZE,
                      NULL,
                      configMAX_PRIORITIES - 2,
                      bg_task_stack,
                      &bg_task_tcb);


  xTimerStart(led_blink_timer, 0);

  /* Start the scheduler. */
  vTaskStartScheduler();

  // Should not reach here
  return 0;
}

//--------------------------------------------------------------------------------
// Background Task
//--------------------------------------------------------------------------------
void bg_task( void *parameters ) {

  struct board_descriptor *brd = board_get_handler();

  // spi_enable(&brd->spi4);
  // gpio_pin_set(&brd->io.spi4_gd_cs);

  // drv835x_drive_enable(&brd->gate_driver);

  vTaskDelay(1);

  // drv835x_set_hs_gate_drive_strength(&brd->gate_driver, DRV835X_IDRIVEP_1000MA, DRV835X_IDRIVEN_2000MA);
  // drv835x_set_ls_gate_drive_strength(&brd->gate_driver, DRV835X_IDRIVEP_1000MA, DRV835X_IDRIVEN_2000MA);

  // drv835x_clear_faults(&brd->gate_driver);

  char *msg = "Hello World!\r\n";
  while (1) {

    // drv835x_read_faults(&brd->gate_driver);
    for (int i = 0; i < strlen(msg); i++) {
      // xQueueSend(serial_queue, &msg[i], 0);
      uart_write_byte(&brd->lpuart1, msg[i]);
    }

    vTaskDelay(100);
  }

}

//--------------------------------------------------------------------------------
// USB Device Task
//--------------------------------------------------------------------------------

void usbd_task( void *parameters )
{   
  /* Unused parameters. */
    ( void ) parameters;

  // usb_device_init();
  cli_usb_init();

  while (1) {
    tud_task();
    tud_cdc_write_flush();
    vTaskDelay(4);
  }

}

//--------------------------------------------------------------------+
// USB CDC Device Callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void) {
// Invoked when device is mounted

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

  shell_init();

  while (1) {

    shell_update();
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
// BLINKING TASK
//--------------------------------------------------------------------+

static void led_blink_cb(TimerHandle_t xTimer)
{   
  struct board_descriptor *brd = board_get_handler();
  /* Unused parameters. */
  ( void ) xTimer;
  gpio_pin_toggle(&brd->io.led_green);
  gpio_pin_toggle(&brd->io.led_red);
  gpio_pin_toggle(&brd->io.led_blue);

}
/*-----------------------------------------------------------*/

void _init(void) {
  // printf("init\r\n");
}
