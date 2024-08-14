/*--------------------------------------------------------
 *  DRV8354 SPI Driver
 *
 *--------------------------------------------------------*/
#pragma once
#include <stdint.h>
#include <stddef.h>

enum drv835x_state {
  DRV835X_STATE_DISABLED,
  DRV835X_STATE_ENABLED,
  DRV835X_STATE_FAULTED,
};

enum drv835x_idriven {
  DRV835X_IDRIVEN_100MA_0,
  DRV835X_IDRIVEN_100MA,
  DRV835X_IDRIVEN_200MA,
  DRV835X_IDRIVEN_300MA,
  DRV835X_IDRIVEN_600MA,
  DRV835X_IDRIVEN_700MA,
  DRV835X_IDRIVEN_800MA,
  DRV835X_IDRIVEN_900MA,
  DRV835X_IDRIVEN_1100MA,
  DRV835X_IDRIVEN_1200MA,
  DRV835X_IDRIVEN_1300MA,
  DRV835X_IDRIVEN_1400MA,
  DRV835X_IDRIVEN_1700MA,
  DRV835X_IDRIVEN_1800MA,
  DRV835X_IDRIVEN_1900MA,
  DRV835X_IDRIVEN_2000MA,
};

enum drv835x_idrivep {
  DRV835X_IDRIVEP_50MA_0,
  DRV835X_IDRIVEP_50MA,
  DRV835X_IDRIVEP_100MA,
  DRV835X_IDRIVEP_150MA,
  DRV835X_IDRIVEP_300MA,
  DRV835X_IDRIVEP_350MA,
  DRV835X_IDRIVEP_400MA,
  DRV835X_IDRIVEP_450MA,
  DRV835X_IDRIVEP_550MA,
  DRV835X_IDRIVEP_600MA,
  DRV835X_IDRIVEP_650MA,
  DRV835X_IDRIVEP_700MA,
  DRV835X_IDRIVEP_850MA,
  DRV835X_IDRIVEP_900MA,
  DRV835X_IDRIVEP_950MA,
  DRV835X_IDRIVEP_1000MA,
};


struct drv835x_interface {
  uint8_t (*drive_enable)(uint8_t state);
  uint8_t (*spi_transfer)(uint16_t in, uint16_t *out);
};

struct drv835x {
  struct drv835x_interface *io;
  uint16_t state;
  uint16_t status;
  uint16_t vgs_status;
};

void drv835x_init(struct drv835x *self);
void drv835x_drive_enable(struct drv835x *self);
void drv835x_drive_disable(struct drv835x *self);
void drv835x_read_faults(struct drv835x *self);
void drv835x_clear_faults(struct drv835x *self);
void drv835x_set_hs_gate_drive_strength(struct drv835x *self, enum drv835x_idrivep idrvp, enum drv835x_idriven idrvn);
void drv835x_set_ls_gate_drive_strength(struct drv835x *self, enum drv835x_idrivep idrvp, enum drv835x_idriven idrvn); 










