#include "imu/lsm6dsow.h"

#include "imu/imu_config.h"

static HAL_StatusTypeDef lsm6dsow_write_reg(lsm6dsow_device_t *device, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(device->bus, device->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef lsm6dsow_read_reg(lsm6dsow_device_t *device, uint8_t reg, uint8_t *value, uint16_t len)
{
  return HAL_I2C_Mem_Read(device->bus, device->address, reg, I2C_MEMADD_SIZE_8BIT, value, len, HAL_MAX_DELAY);
}

static int16_t lsm6dsow_read_i16(const uint8_t *bytes)
{
  return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

HAL_StatusTypeDef lsm6dsow_init(lsm6dsow_device_t *device, I2C_HandleTypeDef *bus, uint16_t address)
{
  uint8_t who_am_i = 0U;
  uint8_t reset = 0x01U;

  device->bus = bus;
  device->address = address;
  device->initialized = 0U;
  device->who_am_i = 0U;

  if (HAL_I2C_IsDeviceReady(bus, address, 5U, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }

  if (lsm6dsow_read_reg(device, LSM6DSOW_WHO_AM_I_REG, &who_am_i, 1U) != HAL_OK) {
    return HAL_ERROR;
  }

  device->who_am_i = who_am_i;
  if (who_am_i != LSM6DSOW_WHO_AM_I_VALUE) {
    return HAL_ERROR;
  }

  if (lsm6dsow_write_reg(device, LSM6DSOW_CTRL3_C_REG, reset) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(20U);

  if (lsm6dsow_write_reg(device, LSM6DSOW_CTRL3_C_REG, PALM_LSM6DSOW_CTRL3_C_INIT) != HAL_OK) {
    return HAL_ERROR;
  }

  if (lsm6dsow_write_reg(device, LSM6DSOW_CTRL1_XL_REG, PALM_LSM6DSOW_ACCEL_CTRL) != HAL_OK) {
    return HAL_ERROR;
  }

  if (lsm6dsow_write_reg(device, LSM6DSOW_CTRL2_G_REG, PALM_LSM6DSOW_GYRO_CTRL) != HAL_OK) {
    return HAL_ERROR;
  }

  device->initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef lsm6dsow_read_sample(lsm6dsow_device_t *device, imu_sample_t *sample)
{
  uint8_t gyro_bytes[6];
  uint8_t accel_bytes[6];

  if ((device->initialized == 0U) || (sample == NULL)) {
    return HAL_ERROR;
  }

  if (lsm6dsow_read_reg(device, LSM6DSOW_OUTX_L_G_REG, gyro_bytes, sizeof(gyro_bytes)) != HAL_OK) {
    return HAL_ERROR;
  }

  if (lsm6dsow_read_reg(device, LSM6DSOW_OUTX_L_A_REG, accel_bytes, sizeof(accel_bytes)) != HAL_OK) {
    return HAL_ERROR;
  }

  sample->gyro_dps.x = (float)lsm6dsow_read_i16(&gyro_bytes[0]) * PALM_LSM6DSOW_GYRO_DPS_PER_LSB;
  sample->gyro_dps.y = (float)lsm6dsow_read_i16(&gyro_bytes[2]) * PALM_LSM6DSOW_GYRO_DPS_PER_LSB;
  sample->gyro_dps.z = (float)lsm6dsow_read_i16(&gyro_bytes[4]) * PALM_LSM6DSOW_GYRO_DPS_PER_LSB;

  sample->accel_g.x = (float)lsm6dsow_read_i16(&accel_bytes[0]) * PALM_LSM6DSOW_ACCEL_G_PER_LSB;
  sample->accel_g.y = (float)lsm6dsow_read_i16(&accel_bytes[2]) * PALM_LSM6DSOW_ACCEL_G_PER_LSB;
  sample->accel_g.z = (float)lsm6dsow_read_i16(&accel_bytes[4]) * PALM_LSM6DSOW_ACCEL_G_PER_LSB;
  sample->timestamp_ms = HAL_GetTick();

  return HAL_OK;
}
