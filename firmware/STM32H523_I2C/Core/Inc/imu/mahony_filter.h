#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <stddef.h>

#include "main.h"
#include "imu/imu_types.h"

typedef struct {
  imu_quatf_t quaternion;
  imu_vec3f_t integral_error;
  float two_kp;
  float two_ki;
  uint16_t warmup_remaining;
  uint8_t initialized;
  imu_vec3f_t last_accel_g;
  uint16_t recovery_counter;

  float dynamic_bias_x;
  float dynamic_bias_y;
  float dynamic_bias_z;
} mahony_filter_t;

HAL_StatusTypeDef mahony_filter_init(mahony_filter_t *filter);
HAL_StatusTypeDef mahony_filter_update(mahony_filter_t *filter,
                                       const imu_sample_t *sample,
                                       float delta_time_s,
                                       imu_attitude_t *attitude);
extern uint8_t  dbg_gravity_trusted;
extern float dbg_comp_x;
extern float dbg_comp_y;
extern float dbg_comp_z;

#endif /* MAHONY_FILTER_H */
