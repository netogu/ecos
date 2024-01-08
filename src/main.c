#include <stdint.h>
// #include <stdio.h>
#include "external/printf.h"
#include "board/bsp.h"
#include "board/shell.h"
#include "tusb.h"
// #include "ush_config.h"

// USB Device IRQ Monitor
extern int usb_lp_irq_counter;

// Scheduler Task
typedef struct task {
  uint32_t period;
  uint32_t elapsed_time;
  void (*callback)(void);
} task_t;

void led_task(void) {
  gpio_pin_toggle(&gpios.led_green);
}

// task_t tasks[] = {
//     {.period = 1, .elapsed_time = 0, .callback = &tud_task},
//   {.period = 1, .elapsed_time = 0, .callback = &shell_task},
//     {.period = 1000, .elapsed_time = 0, .callback = &led_task}
// };

// uint8_t g_task_wait_flag = 0;
// void task_scheduler(task_t *tasks, uint32_t num_tasks) {
//   for (uint32_t i = 0; i < num_tasks; i++) {
//     if (tasks[i].elapsed_time >= tasks[i].period) {
//       tasks[i].callback();
//       tasks[i].elapsed_time = 0;
//     }
//     tasks[i].elapsed_time++;
//   }
//   g_task_wait_flag = 0;
//   while (g_task_wait_flag == 0) {
//     asm("nop");
//   }
// }


int main_loop_counter = 0;
int main(void) {

  board_init();
  shell_setup();
  tusb_init();

 

  while (1) {
    tud_task();
    if (tud_cdc_available()) {
      char buf[32];
      size_t count;
      count=tud_cdc_read(&buf, sizeof(buf));
      printf("read %d bytes\r\n", count);
      printf("buf: %s\r\n", buf);
    }
    printf(".\r\n");
    delay_ms(1);

    // printf("cnt_main, cnt_usb_irq, cdc_available: %d,%d,%d\r\n", 
    //   main_loop_counter, 
    //   usb_lp_irq_counter,
    //   buf_count);

    main_loop_counter++;
  }
}

//--------------------------------------------------------------------+
// USB Mount Callback API (Optional)
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void) {
  // printf("USB mounted\r\n");
  gpio_pin_set(&gpios.led_green);
}

void _init(void) {
  // printf("init\r\n");
}