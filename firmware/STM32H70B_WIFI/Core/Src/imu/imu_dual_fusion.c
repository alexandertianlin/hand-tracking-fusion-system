#include "imu/imu_dual_fusion.h"

#include <math.h>
#include <string.h>

#include "imu/imu_config.h"

static imu_quatf_t imu_dual_fusion_quat_multiply(imu_quatf_t a, imu_quatf_t b)
{
  imu_quatf_t result;

  result.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  result.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  result.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
  result.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

  return result;
}

static imu_quatf_t imu_dual_fusion_quat_normalize(imu_quatf_t quaternion)
{
  float norm = sqrtf((quaternion.w * quaternion.w) +
                     (quaternion.x * quaternion.x) +
                     (quaternion.y * quaternion.y) +
                     (quaternion.z * quaternion.z));

  if (norm <= 0.0f) {
    quaternion.w = 1.0f;
    quaternion.x = 0.0f;
    quaternion.y = 0.0f;
    quaternion.z = 0.0f;
    return quaternion;
  }

  quaternion.w /= norm;
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;

  if (quaternion.w < 0.0f) {
    quaternion.w = -quaternion.w;
    quaternion.x = -quaternion.x;
    quaternion.y = -quaternion.y;
    quaternion.z = -quaternion.z;
  }

  return quaternion;
}

static float imu_dual_fusion_quat_dot(imu_quatf_t a, imu_quatf_t b)
{
  return (a.w * b.w) + (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static imu_quatf_t imu_dual_fusion_apply_alignment(imu_quatf_t alignment, imu_quatf_t input)
{
  return imu_dual_fusion_quat_normalize(imu_dual_fusion_quat_multiply(alignment, input));
}

void imu_dual_fusion_init(imu_dual_fusion_t *fusion)
{
  memset(fusion, 0, sizeof(*fusion));

  fusion->imu0_alignment.w = PALM_IMU0_ALIGN_W;
  fusion->imu0_alignment.x = PALM_IMU0_ALIGN_X;
  fusion->imu0_alignment.y = PALM_IMU0_ALIGN_Y;
  fusion->imu0_alignment.z = PALM_IMU0_ALIGN_Z;

  fusion->imu1_alignment.w = PALM_IMU1_ALIGN_W;
  fusion->imu1_alignment.x = PALM_IMU1_ALIGN_X;
  fusion->imu1_alignment.y = PALM_IMU1_ALIGN_Y;
  fusion->imu1_alignment.z = PALM_IMU1_ALIGN_Z;

  fusion->max_disagreement_deg = PALM_FUSION_MAX_DISAGREEMENT_DEG;
}

imu_dual_fusion_mode_t imu_dual_fusion_update(imu_dual_fusion_t *fusion,
                                              const imu_attitude_t *imu0_attitude,
                                              const imu_attitude_t *imu1_attitude,
                                              imu_attitude_t *fused_attitude,
                                              float *disagreement_deg)
{
  imu_quatf_t q0;
  imu_quatf_t q1;
  float dot = 0.0f;

  fused_attitude->valid = 0U;
  if (disagreement_deg != NULL) {
    *disagreement_deg = 0.0f;
  }

  if ((imu0_attitude->valid == 0U) && (imu1_attitude->valid == 0U)) {
    return IMU_FUSION_NONE;
  }

  if (imu0_attitude->valid != 0U) {
    q0 = imu_dual_fusion_apply_alignment(fusion->imu0_alignment, imu0_attitude->quaternion);
  }

  if (imu1_attitude->valid != 0U) {
    q1 = imu_dual_fusion_apply_alignment(fusion->imu1_alignment, imu1_attitude->quaternion);
  }

  if ((imu0_attitude->valid != 0U) && (imu1_attitude->valid != 0U)) {
    dot = imu_dual_fusion_quat_dot(q0, q1);
    if (dot < 0.0f) {
      q1.w = -q1.w;
      q1.x = -q1.x;
      q1.y = -q1.y;
      q1.z = -q1.z;
      dot = -dot;
    }

    if (dot > 1.0f) {
      dot = 1.0f;
    }

    if (disagreement_deg != NULL) {
      *disagreement_deg = 2.0f * acosf(dot) * (180.0f / 3.14159265f);
    }

    if ((disagreement_deg != NULL) && (*disagreement_deg > fusion->max_disagreement_deg)) {
      fused_attitude->quaternion = q0;
      fused_attitude->euler_deg = imu0_attitude->euler_deg;
      fused_attitude->valid = 1U;
      return IMU_FUSION_IMU0_ONLY;
    }

    fused_attitude->quaternion.w = 0.5f * (q0.w + q1.w);
    fused_attitude->quaternion.x = 0.5f * (q0.x + q1.x);
    fused_attitude->quaternion.y = 0.5f * (q0.y + q1.y);
    fused_attitude->quaternion.z = 0.5f * (q0.z + q1.z);
    fused_attitude->quaternion = imu_dual_fusion_quat_normalize(fused_attitude->quaternion);
    fused_attitude->euler_deg.x = 0.5f * (imu0_attitude->euler_deg.x + imu1_attitude->euler_deg.x);
    fused_attitude->euler_deg.y = 0.5f * (imu0_attitude->euler_deg.y + imu1_attitude->euler_deg.y);
    fused_attitude->euler_deg.z = 0.5f * (imu0_attitude->euler_deg.z + imu1_attitude->euler_deg.z);
    fused_attitude->valid = 1U;
    return IMU_FUSION_BOTH;
  }

  if (imu0_attitude->valid != 0U) {
    fused_attitude->quaternion = q0;
    fused_attitude->euler_deg = imu0_attitude->euler_deg;
    fused_attitude->valid = 1U;
    return IMU_FUSION_IMU0_ONLY;
  }

  fused_attitude->quaternion = q1;
  fused_attitude->euler_deg = imu1_attitude->euler_deg;
  fused_attitude->valid = 1U;
  return IMU_FUSION_IMU1_ONLY;
}
