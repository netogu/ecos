#ifndef ENCODER_H
#define ENCODER_H

/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include "stm32g4_common.h"

typedef struct encoder_t {
    uint32_t (*read)(void);
    void (*load)(uint32_t val);
} encoder_t;

int encoder_init(encoder_t *self);

#endif