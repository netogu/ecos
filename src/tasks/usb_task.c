
#include "bsp.h"
#include "shell.h"
#include "rtos.h"
#include "tiny_printf.h"
#include "task_list.h"

#define PRIORITY_USB_TASK configMAX_PRIORITIES - 3

/*-----------------------------------------------------------*/
// Timers
/*-----------------------------------------------------------*/
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

static TaskHandle_t usb_task_handle;
static StaticTask_t usb_task_tcb;
static StackType_t usb_task_stack[ TASK_STACK_SIZE_USB ];
static void usb_task(void * parameters);

TaskHandle_t usb_task_init(void) {
    // Create the task


    usb_task_handle = xTaskCreateStatic( usb_task,
                        xstr(TASK_NAME_USB),
                        TASK_STACK_SIZE_USB,
                        NULL,
                        PRIORITY_USB_TASK,
                        usb_task_stack,
                        &usb_task_tcb);


    if (usb_task_handle != NULL) {
        // usb_device_init();
        // cli_usb_init();
        tusb_init();
        return usb_task_handle;
    } 
    else {
        return NULL;
    }
}

void usb_task( void *parameters ) {   
  /* Unused parameters. */
    ( void ) parameters;


    while (1) {
        tud_task();
        tud_cdc_write_flush();
        vTaskDelay(TASK_DELAY_USB);
    }

}

//--------------------------------------------------------------------+
// USB CDC Device Callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void) {
// Invoked when device is mounted

//   xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
  
}

// Invoked when device is un-mounted
void tud_umount_cb(void) {

//   xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
  
}
//
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
//   xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  if (tud_mounted()) {
    // xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
  } else {
    // xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
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
