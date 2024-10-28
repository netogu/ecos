

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "hal.h"
#include "tasklist.h"
#include "taskmsg.h"
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

static float fast_fmodf(float x, float y) {
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

static void task_pwm_control(void * parameters) {   

  board_t *brd = board_get_handle();

  /* Unused parameters. */
  ( void ) parameters;


  cordic_t cordic = {
    .function = CORDIC_FUNC_COSINE,
    .cycles = 6,
  };

  cordic_init(&cordic);

  while(1) {

    float valpha;
    float vbeta;
    float va;
    float vb;
    float vc;
    static float vbus = 0.0f;
    static float vd = 0.0f;
    static float vq = 0.0f;
    static uint32_t count = 0;
    static uint32_t count_rate = 0;

    // Check for new messagek
    pwmcon_msg_t msg;
    if (task_pwmcon_msg_receive(&msg)) {
      vq = (float)msg.vq_mv/1000.0f; // Convert to volts from mV
      vd = (float)msg.vd_mv/1000.0f; // Convert to volts from mV
      vbus = (float)msg.vbus_mv/1000.0f; // Convert to volts from mV
      count_rate = msg.count_rate;
    }

    //count = brd->encoder.read();

    gpio_pin_set(&brd->io.test_pin0);
    // float angle_rad = count/1024.0 * 2*PI;
    float angle_rad = count/100000.0f * 2*PI;
    float angle_rad_norm = fast_fmodf(angle_rad, 2.0f*PI) / (2.0f*PI);
    int32_t angle_rad_q31 = f32_to_q31(angle_rad_norm) << 1;

    cordic_write(angle_rad_q31);
    int32_t cos_q31 = cordic_read();
    int32_t sin_q31 = cordic_read();

    float cos_f32 = q31_to_f32(cos_q31);
    float sin_f32 = q31_to_f32(sin_q31);

    arm_inv_park_f32(vd/vbus, vq/vbus, &valpha, &vbeta, sin_f32, cos_f32);
    arm_inv_clarke_f32(valpha, vbeta, &va, &vb);

    // a + b + c = 0
    vc = -va - vb;

    float dc_a = (va + 1.0f)/2.0f;
    float dc_b = (vb + 1.0f)/2.0f;
    float dc_c = (vc + 1.0f)/2.0f;

    pwm_3ph_set_duty(&brd->mcpwm, dc_a, dc_b, dc_c);

    gpio_pin_clear(&brd->io.test_pin0);

    count += count_rate;
    if (count >= 100000) {
      count = 0;
    }

    vTaskDelay(TASK_DELAY_PWM_CONTROL);
  }
}