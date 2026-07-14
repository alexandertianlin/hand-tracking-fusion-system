#ifndef LSM6DSOW_H
#define LSM6DSOW_H

#include "main.h"
#include "imu/imu_types.h"

#define LSM6DSOW_WHO_AM_I_REG 0x0FU
#define LSM6DSOW_CTRL1_XL_REG 0x10U
#define LSM6DSOW_CTRL2_G_REG  0x11U
#define LSM6DSOW_CTRL3_C_REG  0x12U
#define LSM6DSOW_STATUS_REG   0x1EU
#define LSM6DSOW_OUTX_L_G_REG 0x22U
#define LSM6DSOW_OUTX_L_A_REG 0x28U

#define LSM6DSOW_WHO_AM_I_VALUE 0x6CU

typedef struct {
  I2C_HandleTypeDef *bus;
  uint16_t address;
  uint8_t who_am_i;
  uint8_t initialized;
} lsm6dsow_device_t;

HAL_StatusTypeDef lsm6dsow_init(lsm6dsow_device_t *device, I2C_HandleTypeDef *bus, uint16_t address);
HAL_StatusTypeDef lsm6dsow_read_sample(lsm6dsow_device_t *device, imu_sample_t *sample);

#endif /* LSM6DSOW_H */
