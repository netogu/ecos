#ifndef STM32G4_CORDIC_H
#define STM32G4_CORDIC_H

#include "stm32g4_common.h"
#include <math.h>

typedef struct cordic_t {
  enum function {
    CORDIC_FUNC_COSINE = 0,
    CORDIC_FUNC_SINE = 1,
  } function;
  uint8_t cycles;

} cordic_t;

int cordic_init(cordic_t *self);
void cordic_write(uint32_t args);
uint32_t cordic_read(void);
int cordic_result_is_ready(void);

// #define q31_to_f32(x) ldexpf((int32_t) x, -31)
#define q31_to_f32(x) x * 0.0000000004656612873077392578125f

static inline int32_t f32_to_q31(float f32_value) {
  // Clamp the input to the Q31 representable range
  if (f32_value > 1.0f - (1.0f / 2147483648.0f)) {
    f32_value = 1.0f - (1.0f / 2147483648.0f);
  } else if (f32_value < -1.0f) {
    f32_value = -1.0f;
  }

  // Scale and convert to Q31
  return (int32_t)(f32_value * 2147483648.0f);
}

inline float cordic_q31_cosf(float x) {
  (void)x;
  // int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*PI) / (2.0f * PI)) << 1;

  // COSINE FUNC
  // 6cycle precision (24 iterations)
  // SCALE = 0
  // Nbwrite = 1
  // Nbread = 1
  // insize = 32bit
  // outsize = 32bit
  return 0.0f;
}

#endif
