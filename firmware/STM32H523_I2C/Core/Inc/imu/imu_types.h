#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

typedef struct {
  float x;
  float y;
  float z;
} imu_vec3f_t;

typedef struct {
  float w;
  float x;
  float y;
  float z;
} imu_quatf_t;

typedef struct {
  imu_vec3f_t accel_g;
  imu_vec3f_t gyro_dps;
  uint32_t timestamp_ms;
} imu_sample_t;

typedef struct {
  imu_vec3f_t accel_g;
  imu_quatf_t quaternion;
  imu_vec3f_t euler_deg;
  uint8_t valid;
} imu_attitude_t;

#endif /* IMU_TYPES_H */
