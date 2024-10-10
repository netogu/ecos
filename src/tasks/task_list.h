#ifndef TASK_LIST_H
#define TASK_LIST_H

#include <stdbool.h>
#include "rtos.h"

#define str(s) #s
#define xstr(s) str(s)

//--------------------------------------------------------------------+
// Task List
//--------------------------------------------------------------------+

#define TASK_NAME_TASK_MANAGER          taskmanager
#define TASK_NAME_USB                   usb
#define TASK_NAME_CLI                   cli 
#define TASK_NAME_LED                   led
#define TASK_NAME_PWM_CONTROL           pwm_control

#define TASK_PRIORITY_TASK_MANAGER      1
#define TASK_PRIORITY_USB               2
#define TASK_PRIORITY_CLI               1
#define TASK_PRIORITY_LED               4
#define TASK_PRIORITY_PWM_CONTROL       4


#define TASK_REPEAT_TASK_MANAGER        1
#define TASK_REPEAT_USB                 1
#define TASK_REPEAT_CLI                 1
#define TASK_REPEAT_LED                 1
#define TASK_REPEAT_PWM_CONTROL         1

#define TASK_DELAY_TASK_MANAGER         5
#define TASK_DELAY_USB                  4
#define TASK_DELAY_CLI                  1
#define TASK_DELAY_LED                  1000
#define TASK_DELAY_PWM_CONTROL          250

#define TASK_STACK_SIZE_TASK_MANAGER    1024
#define TASK_STACK_SIZE_USB             1024
#define TASK_STACK_SIZE_CLI             1024
#define TASK_STACK_SIZE_LED             1024
#define TASK_STACK_SIZE_PWM_CONTROL     1024

//--------------------------------------------------------------------+
// Task Functions
//--------------------------------------------------------------------+

TaskHandle_t task_manager_init(void);
TaskHandle_t usb_task_init(void);
TaskHandle_t cli_task_init(void);
TaskHandle_t led_task_init(void);
TaskHandle_t task_pwm_control_init(void);

//--------------------------------------------------------------------+
// Task Descriptors
//--------------------------------------------------------------------+

// Task Function pointer typedef
typedef TaskHandle_t (*task_init_t)(void);

typedef struct task_descriptor_s {
    const char * const name;
    const bool startup;
    task_init_t init;  
} task_descriptor_t;

extern const task_descriptor_t task_list[];
extern const size_t task_list_size;

void bootup_system(void);

#endif // TASK_LIST_H