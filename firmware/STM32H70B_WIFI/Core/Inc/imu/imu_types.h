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

typedef enum {
  IMU_AXIS_SOURCE_X = 0U,
  IMU_AXIS_SOURCE_Y = 1U,
  IMU_AXIS_SOURCE_Z = 2U
} imu_axis_source_t;

typedef struct {
  imu_axis_source_t output_x_source;
  imu_axis_source_t output_y_source;
  imu_axis_source_t output_z_source;
  uint8_t invert_x;
  uint8_t invert_y;
  uint8_t invert_z;
} imu_axis_remap_config_t;

typedef struct {
  imu_vec3f_t accel_g;
  imu_vec3f_t gyro_dps;
  uint32_t timestamp_ms;
} imu_sample_t;

typedef struct {
  imu_quatf_t quaternion;
  imu_vec3f_t euler_deg;
  uint8_t valid;
} imu_attitude_t;

#endif /* IMU_TYPES_H */
