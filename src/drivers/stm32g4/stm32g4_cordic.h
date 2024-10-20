#ifndef STM32G4_CORDIC_H
#define STM32G4_CORDIC_H

#include "stm32g4_common.h"
#include <math.h>
#define PI 3.141592653589793

typedef struct cordic_t {
    enum function {
        CORDIC_FUNC_COSINE=0,
        CORDIC_FUNC_SINE=1,
    } function;
    uint8_t cycles; 

} cordic_t;


int cordic_init(cordic_t *self);
void  cordic_write(uint32_t args);
uint32_t  cordic_read(void);
int cordic_result_is_ready(void);

#define q31_to_f32(x) ldexp((int32_t) x, -31)

static inline int f32_to_q31(double input) {
    const float Q31_MAX_F = 0x0.FFFFFFp0F;
    const float Q31_MIN_F = -1.0F;
    return (int)roundf(scalbnf(fmaxf(fminf(input, Q31_MAX_F), Q31_MIN_F), 31));
}

inline
float cordic_q31_cosf(float x) {
    int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*PI) / (2.0f * PI)) << 1;
    int32_t output_q31;
    // COSINE FUNC
    // 6cycle precision (24 iterations)
    // SCALE = 0
    // Nbwrite = 1
    // Nbread = 1
    // insize = 32bit
    // outsize = 32bit

}

#endif