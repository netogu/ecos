/*--------------------------------------------------------
 *  DRV8354 SPI Driver
 *
 *--------------------------------------------------------*/
#include "drivers/power/drv835x.h"

//--------------------------------------------------------
// DRV8353 Register Definitions
//--------------------------------------------------------

#define DRV8353_FAULT_STATUS_REG          0
#define DRV8353_FAULT_STATUS_FAULT_Pos    10
#define DRV8353_FAULT_STATUS_VDS_OCP_Pos  9
#define DRV8353_FAULT_STATUS_GDF_Pos      8
#define DRV8353_FAULT_STATUS_UVLO_Pos     7
#define DRV8353_FAULT_STATUS_OTSD_Pos     6
#define DRV8353_FAULT_STATUS_VDS_HA_Pos   5
#define DRV8353_FAULT_STATUS_VDS_LA_Pos   4
#define DRV8353_FAULT_STATUS_VDS_HB_Pos   3
#define DRV8353_FAULT_STATUS_VDS_LB_Pos   2
#define DRV8353_FAULT_STATUS_VDS_HC_Pos   1
#define DRV8353_FAULT_STATUS_VDS_LC_Pos   0

#define DRV8353_VGS_STATUS_REG            1
#define DRV8353_VGS_STATUS_SA_OC_Pos      10
#define DRV8353_VGS_STATUS_SB_OC_Pos      9
#define DRV8353_VGS_STATUS_SC_OC_Pos      8
#define DRV8353_VGS_STATUS_OTW_Pos        7
#define DRV8353_VGS_STATUS_GDUV_Pos       6
#define DRV8353_VGS_STATUS_VGS_HA_Pos     5
#define DRV8353_VGS_STATUS_VGS_LA_Pos     4
#define DRV8353_VGS_STATUS_VGS_HB_Pos     3
#define DRV8353_VGS_STATUS_VGS_LB_Pos     2
#define DRV8353_VGS_STATUS_VGS_HC_Pos     1
#define DRV8353_VGS_STATUS_VGS_LC_Pos     0

#define DRV8353_DRIVER_CTRL_REG           2
#define DRV8353_DRIVER_CTRL_OCP_ACT_Pos   10
#define DRV8353_DRIVER_CTRL_DIS_GDUV_Pos  9
#define DRV8353_DRIVER_CTRL_DIS_GDF_Pos   8
#define DRV8353_DRIVER_CTRL_OTW_REP_Pos   7
#define DRV8353_DRIVER_CTRL_PWM_MODE_Pos  5
#define DRV8353_DRIVER_CTRL_1PWM_COM_Pos  4
#define DRV8353_DRIVER_CTRL_1PWM_DIR_Pos  3
#define DRV8353_DRIVER_CTRL_COAST_Pos     2
#define DRV8353_DRIVER_CTRL_BRAKE_Pos     1
#define DRV8353_DRIVER_CTRL_CLR_FAULT_Pos 0

#define DRV8353_GD_HS_REG                 3
#define DRV8353_GD_HS_LOCK_Pos            8
#define DRV8353_GD_HS_IDRIVEP_HS_Pos      4
#define DRV8353_GD_HS_IDRIVEN_HS_Pos      0

#define DRV8353_GD_LS_REG                 4
#define DRV8353_GD_LS_CBC_Pos             10
#define DRV8353_GD_LS_TDRIVE_Pos          8
#define DRV8353_GD_LS_IDRIVEP_LS_Pos      4
#define DRV8353_GD_LS_IDRIVEN_LS_Pos      0

#define DRV8353_OCP_CTRL_REG              5
#define DRV8353_OCP_CTRL_TRETRY_Pos       10
#define DRV8353_OCP_CTRL_DEAD_TIME_Pos    8
#define DRV8353_OCP_CTRL_OCP_MODE_Pos     6
#define DRV8353_OCP_CTRL_OCP_DEG_Pos      4
#define DRV8353_OCP_CTRL_VDS_LVL_Pos      0

#define DRV8353_CSA_CTRL_REG              6
#define DRV8353_CSA_CTRL_CSA_FET_Pos      10
#define DRV8353_CSA_CTRL_VREF_DIV_Pos     9
#define DRV8353_CSA_CTRL_LS_REF_Pos       8
#define DRV8353_CSA_CTRL_CSA_GAIN_Pos     6
#define DRV8353_CSA_CTRL_DIS_SEN_Pos      5
#define DRV8353_CSA_CTRL_CSA_CAL_A_Pos    4
#define DRV8353_CSA_CTRL_CSA_CAL_B_Pos    3
#define DRV8353_CSA_CTRL_CSA_CAL_C_Pos    2
#define DRV8353_CSA_CTRL_SEN_LVL_Pos      0

#define DRV8353_DRV_CONFIG_REG            7
#define DRV8353_DRV_CONFIG_CAL_MODE_Pos   0

#define DRV8353_FRAME(rw, addr, data) ((rw << 15) | (addr << 11) | (data))

void drv835x_init(struct drv835x *self);

void drv835x_drive_enable(struct drv835x *self) {
  self->drive_enable(1);
}

void drv835x_drive_disable(struct drv835x *self) {
  self->drive_enable(0);
}

void drv835x_read_faults(struct drv835x *self) {
  uint16_t frame = DRV8353_FRAME(0, DRV8353_FAULT_STATUS_REG, 0);
  self->spi_transfer(frame, &self->status);
  frame = DRV8353_FRAME(0, DRV8353_VGS_STATUS_REG, 0);
  self->spi_transfer(frame, &self->vgs_status);
}

void drv835x_clear_faults(struct drv835x *self) {
  uint16_t frame = DRV8353_FRAME(1, DRV8353_DRIVER_CTRL_REG, 1 << DRV8353_DRIVER_CTRL_CLR_FAULT_Pos);
  uint16_t data_rx;
  self->spi_transfer(frame, &data_rx);
}

void drv835x_set_hs_gate_drive_strength(struct drv835x *self, enum drv835x_idrivep idrvp, enum drv835x_idriven idrvn){
  uint16_t data = (idrvp << DRV8353_GD_HS_IDRIVEP_HS_Pos) | (idrvn << DRV8353_GD_HS_IDRIVEN_HS_Pos);
  uint16_t frame = DRV8353_FRAME(1, DRV8353_GD_HS_REG, data);
  uint16_t data_rx;
  self->spi_transfer(frame, &data_rx);
}

void drv835x_set_ls_gate_drive_strength(struct drv835x *self, enum drv835x_idrivep idrvp, enum drv835x_idriven idrvn){
  uint16_t data = (idrvp << DRV8353_GD_LS_IDRIVEP_LS_Pos) | (idrvn << DRV8353_GD_LS_IDRIVEN_LS_Pos);
  uint16_t frame = DRV8353_FRAME(1, DRV8353_GD_LS_REG, data);
  uint16_t data_rx;
  self->spi_transfer(frame, &data_rx);
}










