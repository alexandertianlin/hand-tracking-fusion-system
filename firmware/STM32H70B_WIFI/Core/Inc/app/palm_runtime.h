#ifndef PALM_RUNTIME_H
#define PALM_RUNTIME_H

#include "main.h"
#include "imu/imu_config.h"
#include "imu/mahony_filter.h"
#include "imu/imu_types.h"
#include "imu/lsm6dsow.h"

typedef struct {
  lsm6dsow_device_t imu_devices[PALM_IMU_COUNT];
  mahony_filter_t mahony;
  imu_attitude_t fused_attitude;
  imu_vec3f_t gyro_bias_accum_dps;
  imu_vec3f_t gyro_bias_dps;
  imu_quatf_t zero_reference;
  uint32_t last_update_ms;
  uint32_t frames_sent;
  uint32_t usb_busy_count;
  uint16_t calibration_samples;
  uint16_t calibration_attempts;
  uint8_t calibration_complete;
  uint8_t zero_pending;
  uint8_t zero_captured;
  uint8_t status;
  uint8_t initialized;
} palm_runtime_t;

HAL_StatusTypeDef palm_runtime_init(palm_runtime_t *runtime, I2C_HandleTypeDef *bus);
void palm_runtime_process(palm_runtime_t *runtime);

/* Palm-side runtime "Zero All" reference layer for the palm USB output.
 * Request: arm a capture; the next fused attitude sample becomes the zero
 * reference, and subsequent palm USB frames emit (inv(ref) * q_fused).
 * Clear: drop the reference and resume raw fused output. */
void palm_runtime_request_zero(palm_runtime_t *runtime);
void palm_runtime_clear_zero(palm_runtime_t *runtime);

#endif /* PALM_RUNTIME_H */
