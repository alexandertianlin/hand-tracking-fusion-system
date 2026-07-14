 #ifndef IMU_DUAL_FUSION_H
#define IMU_DUAL_FUSION_H

#include "imu/imu_types.h"

typedef enum {
  IMU_FUSION_NONE = 0,
  IMU_FUSION_BOTH = 1,
  IMU_FUSION_IMU0_ONLY = 2,
  IMU_FUSION_IMU1_ONLY = 3
} imu_dual_fusion_mode_t;

typedef struct {
  imu_quatf_t imu0_alignment;
  imu_quatf_t imu1_alignment;
  float max_disagreement_deg;
} imu_dual_fusion_t;

void imu_dual_fusion_init(imu_dual_fusion_t *fusion);
imu_dual_fusion_mode_t imu_dual_fusion_update(imu_dual_fusion_t *fusion,
                                              const imu_attitude_t *imu0_attitude,
                                              const imu_attitude_t *imu1_attitude,
                                              imu_attitude_t *fused_attitude,
                                              float *disagreement_deg);

#endif /* IMU_DUAL_FUSION_H */
