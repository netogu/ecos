#ifndef ENCODER_H
#define ENCODER_H

/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include <stdint.h>

typedef struct encoder_t {
    uint32_t (*read)(void);
} encoder_t;

int encoder_init(encoder_t *self);

#endif