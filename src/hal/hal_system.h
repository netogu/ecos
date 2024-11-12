#ifndef __HAL_SYSTEM_H
#define __HAL_SYSTEM_H

#include <stdint.h>


typedef struct system_id_s {
    uint32_t device_id;
    uint16_t flash_size_kb;
    uint16_t package_type;
    uint32_t uid[3];
} system_id_t;

void system_get_id(system_id_t *id);


#endif // __HAL_SYSTEM_H