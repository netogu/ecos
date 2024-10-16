

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "hal.h"
#include "tasklist.h"

static TaskHandle_t task_pwm_control_handle;
static StaticTask_t task_pwm_control_tcb;
static StackType_t task_pwm_control_stack[ TASK_STACK_SIZE_LED ];
static void task_pwm_control(void * parameters);

TaskHandle_t task_pwm_control_init(void) {
    // Create the task

    task_pwm_control_handle = xTaskCreateStatic( task_pwm_control,
                        xstr(TASK_NAME_PWM_CONTROL),
                        TASK_STACK_SIZE_PWM_CONTROL,
                        NULL,
                        TASK_PRIORITY_PWM_CONTROL,
                        task_pwm_control_stack,
                        &task_pwm_control_tcb);

    if (task_pwm_control_handle != NULL) {
        return task_pwm_control_handle;
    } 
    else {
        return NULL;
    }
}

static void task_pwm_control(void * parameters) {   

  board_t *brd = board_get_handle();

  /* Unused parameters. */
  ( void ) parameters;


  while(1) {

    uint32_t count = brd->encoder.read();
    pwm_set_duty(&brd->mcpwm.pwma, count/1024.0);

    // cli_printf("\r\nEncoder Count : %u", count);

    vTaskDelay(TASK_DELAY_PWM_CONTROL);
  }
}