

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "hal.h"
#include "tasklist.h"
#include "math.h"

#define ANGLE_CORDIC      (int32_t)0x10000000       /* pi/8 in CORDIC input angle mapping */
#define MODULUS           (int32_t)0x7FFFFFFF       /* 1 */
#define COS_REF           (int32_t)0x7641AF3C       /* cos(pi/8) reference value */
#define SIN_REF           (int32_t)0x30FBC54D       /* sin(pi/8) reference value */
#define DELTA             (int32_t)0x00001000       /* Max residual error for cos and sin, with 6 cycle precision:
                                                       2^-19 max residual error, ie 31-19=12 LSB, ie <0x1000 */
#define COS_120D          (float) -0.50
#define SIN_120D          (float) +0.866025404
#define COS_240D          COS_120D
#define SIN_240D          -SIN_120D


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


  cordic_t cordic = {
    .function = CORDIC_FUNC_COSINE,
    .cycles = 6,
  };

  cordic_init(&cordic);
  uint32_t result = 0;



  while(1) {

    uint32_t count = brd->encoder.read();

    // float angle_rad = count/1024.0 * 2*PI;
    float angle_rad = count/400.0 * 2*PI;
    int32_t angle_rad_q31 = f32_to_q31(fmod(angle_rad, 2*PI) / (2*PI)) << 1;
    cordic_write(angle_rad_q31);
    int32_t cos_q31 = cordic_read();
    int32_t sin_q31 = cordic_read();
    float cos = q31_to_f32(cos_q31);
    float sin = q31_to_f32(sin_q31);
    float cos_120 = cos*COS_120D - sin*SIN_120D;
    float cos_240 = cos*COS_120D - sin*SIN_240D;



    // cli_printf("\r\ncount=%u\tangle=%x\tcos(angle)=%x\tsin(angle)=%x", count, angle_rad_q31, cos_q31, sin_q31);


    
    float duty = count/1024.0;
    // float dutya = fabs(cos);
    // float dutyb = fabs(cos_120);
    // float dutyc = fabs(cos_240);
    float dutya = (cos + 1.0)/2.0;
    float dutyb = (cos_120 + 1.0)/2.0;
    float dutyc = (cos_240 + 1.0)/2.0;
    // pwm_set_duty(&brd->mcpwm.pwma, count/1024.0);
    pwm_3ph_set_duty(&brd->mcpwm, dutya, dutyb, dutyc);

    // cli_printf("\r\nEncoder Count : %u", count);

    vTaskDelay(TASK_DELAY_PWM_CONTROL);
  }
}