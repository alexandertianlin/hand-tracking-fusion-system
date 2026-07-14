#ifndef FINGERTIP_RUNTIME_H
#define FINGERTIP_RUNTIME_H

#include "main.h"
#include "imu/imu_config.h"
#include "imu/imu_types.h"
#include "imu/lsm6dsow.h"
#include "imu/mahony_filter.h"

typedef struct {
  lsm6dsow_device_t imu_devices[PALM_IMU_COUNT];
  mahony_filter_t mahony;
  imu_attitude_t fused_attitude;
  imu_vec3f_t gyro_bias_accum_dps;
  imu_vec3f_t gyro_bias_dps;
  I2C_HandleTypeDef *imu_bus;
  UART_HandleTypeDef *tx_uart;
  uint32_t last_update_ms;
  uint32_t frames_sent;
  uint32_t frames_dropped;
  uint16_t calibration_samples;
  uint16_t calibration_attempts;
  uint8_t tx_frame[PALM_PROTOCOL_FRAME_SIZE];
  uint8_t calibration_complete;
  uint8_t status;
  volatile uint8_t tx_busy;
  uint8_t transport_busy_latched;
  uint8_t initialized;
} fingertip_runtime_t;

HAL_StatusTypeDef fingertip_runtime_init(fingertip_runtime_t *runtime,
                                         I2C_HandleTypeDef *imu_bus,
                                         UART_HandleTypeDef *tx_uart);
void fingertip_runtime_process(fingertip_runtime_t *runtime);
void fingertip_runtime_on_uart_tx_complete(fingertip_runtime_t *runtime,
                                           UART_HandleTypeDef *huart);

#endif /* FINGERTIP_RUNTIME_H */
