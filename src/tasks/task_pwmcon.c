

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "hal.h"
#include "tasklist.h"
#include "math.h"
#include "arm_math.h"

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

  float fast_fmodf(float x, float y) {
      if (y == 0.0f) {
          return NAN; // Return NaN for undefined behavior
      }

      // Calculate the integer multiple of y closest to x
      float quotient = (int)(x / y);  // Cast to int truncates toward zero
      float result = x - quotient * y;

      // Adjust result if it goes out of range due to truncation
      if (result < 0.0f && y > 0.0f) {
          result += y;
      } else if (result > 0.0f && y < 0.0f) {
          result += y;
      }

      return result;
  }

  while(1) {

    float ialpha;
    float ibeta;
    float id = 0.0f;
    float iq = 1.0f;
    float ia;
    float ib;
    float ic;
    static uint32_t count = 0;

    //count = brd->encoder.read();

    gpio_pin_set(&brd->io.test_pin0);
    // float angle_rad = count/1024.0 * 2*PI;
    float angle_rad = count/4096.0f * 2*PI;
    float angle_rad_norm = fast_fmodf(angle_rad, 2.0f*PI) / (2.0f*PI);
    int32_t angle_rad_q31 = f32_to_q31(angle_rad_norm) << 1;

    cordic_write(angle_rad_q31);
    int32_t cos_q31 = cordic_read();
    int32_t sin_q31 = cordic_read();

    float cos = q31_to_f32(cos_q31);
    float sin = q31_to_f32(sin_q31);

    arm_inv_park_f32(id, iq, &ialpha, &ibeta, sin, cos);
    arm_inv_clarke_f32(ialpha, ibeta, &ia, &ib);

    ic = -ia - ib;
    (void)ic;
    gpio_pin_clear(&brd->io.test_pin0);
    // float cos_120 = cos*COS_120D - sin*SIN_120D;
    // float cos_240 = cos*COS_120D - sin*SIN_240D;



    // cli_printf("\r\ncount=%u\tangle=%x\tcos(angle)=%x\tsin(angle)=%x", count, angle_rad_q31, cos_q31, sin_q31);


    
    // float duty = count/1024.0;

    // float dutya = fabs(cos);
    // float dutyb = fabs(cos_120);
    // float dutyc = fabs(cos_240);
    // float dutya = (cos + 1.0f)/2.0f;
    // float dutyb = (cos_120 + 1.0f)/2.0f;
    // float dutyc = (cos_240 + 1.0f)/2.0f;
    float dutya = (ia + 1.0f)/2.0f;
    float dutyb = (ib + 1.0f)/2.0f;
    float dutyc = (ic + 1.0f)/2.0f;
    // pwm_set_duty(&brd->mcpwm.pwma, count/1024.0);
    pwm_3ph_set_duty(&brd->mcpwm, dutya, dutyb, dutyc);

    // cli_printf("\r\nEncoder Count : %u", count);

    count += 1;
    if (count >= 4096) {
      count = 0;
    }

    vTaskDelay(TASK_DELAY_PWM_CONTROL);
  }
}