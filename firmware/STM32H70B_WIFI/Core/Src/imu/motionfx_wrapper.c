#include "imu/motionfx_wrapper.h"

#include <math.h>
#include <string.h>

#include "imu/imu_config.h"

volatile uint32_t g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_IDLE;
volatile uint32_t g_motionfx_diag_last_status = HAL_OK;
volatile uint32_t g_motionfx_diag_state_size = 0U;

static void motionfx_wrapper_copy_orientation(char dst[4], const char *src)
{
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = '\0';
}

static void motionfx_wrapper_normalize_quaternion(imu_quatf_t *quaternion)
{
  float norm = sqrtf((quaternion->w * quaternion->w) +
                     (quaternion->x * quaternion->x) +
                     (quaternion->y * quaternion->y) +
                     (quaternion->z * quaternion->z));

  if (norm <= 0.0f) {
    quaternion->w = 1.0f;
    quaternion->x = 0.0f;
    quaternion->y = 0.0f;
    quaternion->z = 0.0f;
    return;
  }

  quaternion->w /= norm;
  quaternion->x /= norm;
  quaternion->y /= norm;
  quaternion->z /= norm;

  if (quaternion->w < 0.0f) {
    quaternion->w = -quaternion->w;
    quaternion->x = -quaternion->x;
    quaternion->y = -quaternion->y;
    quaternion->z = -quaternion->z;
  }
}

HAL_StatusTypeDef motionfx_wrapper_init(motionfx_wrapper_t *wrapper,
                                        const char *acc_orientation,
                                        const char *gyro_orientation)
{
  memset(wrapper, 0, sizeof(*wrapper));

  g_motionfx_diag_last_status = HAL_ERROR;
  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_ENABLE_CRC;
  __HAL_RCC_CRC_CLK_ENABLE();

  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_GET_SIZE;
  wrapper->required_state_bytes = (uint16_t)MotionFX_GetStateSize();
  g_motionfx_diag_state_size = wrapper->required_state_bytes;
  if (wrapper->required_state_bytes > MOTIONFX_WRAPPER_STATE_BYTES) {
    g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_FAILED;
    return HAL_ERROR;
  }

  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_INITIALIZE;
  MotionFX_initialize((MFXState_t)wrapper->state_words);

  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_GET_KNOBS;
  MotionFX_getKnobs((MFXState_t)wrapper->state_words, &wrapper->knobs);

  motionfx_wrapper_copy_orientation(wrapper->knobs.acc_orientation, acc_orientation);
  motionfx_wrapper_copy_orientation(wrapper->knobs.gyro_orientation, gyro_orientation);
  motionfx_wrapper_copy_orientation(wrapper->knobs.mag_orientation, acc_orientation);

  wrapper->knobs.output_type = MFX_ENGINE_OUTPUT_ENU;
  wrapper->knobs.LMode = 1U;
  wrapper->knobs.modx = 1U;
  wrapper->knobs.gbias_acc_th_sc = PALM_MOTIONFX_GBIAS_ACC_TH_SC;
  wrapper->knobs.gbias_gyro_th_sc = PALM_MOTIONFX_GBIAS_GYRO_TH_SC;
  wrapper->knobs.gbias_mag_th_sc = PALM_MOTIONFX_GBIAS_MAG_TH_SC;
  wrapper->knobs.start_automatic_gbias_calculation = 1;

  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_SET_KNOBS;
  MotionFX_setKnobs((MFXState_t)wrapper->state_words, &wrapper->knobs);

  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_ENABLE_6X;
  MotionFX_enable_6X((MFXState_t)wrapper->state_words, MFX_ENGINE_ENABLE);

  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_ENABLE_9X;
  MotionFX_enable_9X((MFXState_t)wrapper->state_words, MFX_ENGINE_DISABLE);

  wrapper->warmup_remaining = PALM_IMU_WARMUP_SAMPLES;
  wrapper->initialized = 1U;
  g_motionfx_diag_last_status = HAL_OK;
  g_motionfx_diag_stage = MOTIONFX_DIAG_STAGE_READY;

  return HAL_OK;
}

HAL_StatusTypeDef motionfx_wrapper_update(motionfx_wrapper_t *wrapper,
                                          const imu_sample_t *sample,
                                          float delta_time_s,
                                          imu_attitude_t *attitude)
{
  MFX_input_t data_in;
  MFX_output_t data_out;

  if ((wrapper->initialized == 0U) || (sample == NULL) || (attitude == NULL)) {
    return HAL_ERROR;
  }

  if (delta_time_s <= 0.0f) {
    delta_time_s = PALM_IMU_FILTER_DT_S;
  }

  memset(&data_in, 0, sizeof(data_in));
  memset(&data_out, 0, sizeof(data_out));

  data_in.acc[0] = sample->accel_g.x;
  data_in.acc[1] = sample->accel_g.y;
  data_in.acc[2] = sample->accel_g.z;

  data_in.gyro[0] = sample->gyro_dps.x;
  data_in.gyro[1] = sample->gyro_dps.y;
  data_in.gyro[2] = sample->gyro_dps.z;

  MotionFX_propagate((MFXState_t)wrapper->state_words, &data_out, &data_in, &delta_time_s);
  MotionFX_update((MFXState_t)wrapper->state_words, &data_out, &data_in, &delta_time_s, NULL);

  attitude->quaternion.w = data_out.quaternion[0];
  attitude->quaternion.x = data_out.quaternion[1];
  attitude->quaternion.y = data_out.quaternion[2];
  attitude->quaternion.z = data_out.quaternion[3];
  attitude->euler_deg.x = data_out.rotation[0];
  attitude->euler_deg.y = data_out.rotation[1];
  attitude->euler_deg.z = data_out.rotation[2];

  motionfx_wrapper_normalize_quaternion(&attitude->quaternion);

  if (wrapper->warmup_remaining > 0U) {
    wrapper->warmup_remaining--;
    attitude->valid = 0U;
    return HAL_BUSY;
  }

  attitude->valid = 1U;
  return HAL_OK;
}
