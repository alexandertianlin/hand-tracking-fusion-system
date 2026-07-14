#ifndef PALM_PROTOCOL_H
#define PALM_PROTOCOL_H

#include "imu/imu_types.h"
#include "imu/imu_config.h"

typedef struct {
  imu_vec3f_t accel_g;
  imu_quatf_t orientation;
  uint8_t status;
  uint8_t node_id;
  uint8_t model_id;

} palm_protocol_payload_t;

typedef struct {
  imu_sample_t imu_samples[PALM_IMU_COUNT];
  uint8_t status;
  uint8_t sequence;
} palm_protocol_raw_payload_t;

void palm_protocol_build_frame(const palm_protocol_payload_t *payload,
                               uint8_t frame[PALM_PROTOCOL_FRAME_SIZE]);
void palm_protocol_build_raw_frame(const palm_protocol_raw_payload_t *payload,
                                   uint8_t frame[PALM_RAW_PROTOCOL_FRAME_SIZE]);

#endif /* PALM_PROTOCOL_H */
