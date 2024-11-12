#include "hal_system.h"
#include "stm32g4.h"

void system_get_id(system_id_t *id) {

    if (id == NULL) {
        return;
    }

    // Read device ID
    id->device_id = DBGMCU->IDCODE & 0x00000FFF;
    // read flash size in KB
    id->flash_size_kb = *(uint16_t *)FLASHSIZE_BASE;

    // read unique device ID
    id->uid[0] = *(uint32_t *)UID_BASE;
    id->uid[1] = *(uint32_t *)(UID_BASE + 4);
    id->uid[2] = *(uint32_t *)(UID_BASE + 8);

    // read package size
    id->package_type = *(uint16_t *)PACKAGE_BASE;

}
