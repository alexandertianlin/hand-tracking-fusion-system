#include "app/palm_runtime.h"

#include <math.h>
#include <string.h>

#include "imu/imu_config.h"
#include "protocol/palm_protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

static const imu_axis_remap_config_t g_palm_imu_axis_remaps[PALM_IMU_COUNT] = {
  PALM_IMU0_AXIS_REMAP_INIT,
  PALM_IMU1_AXIS_REMAP_INIT
};

static void palm_runtime_reset_identity(imu_attitude_t *attitude)
{
  memset(attitude, 0, sizeof(*attitude));
  attitude->quaternion.w = 1.0f;
}

static void palm_runtime_zero_reference_identity(imu_quatf_t *quaternion)
{
  quaternion->w = 1.0f;
  quaternion->x = 0.0f;
  quaternion->y = 0.0f;
  quaternion->z = 0.0f;
}

static imu_quatf_t palm_runtime_quat_multiply(imu_quatf_t a, imu_quatf_t b)
{
  imu_quatf_t result;

  result.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  result.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  result.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
  result.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

  return result;
}

static imu_quatf_t palm_runtime_quat_conjugate(imu_quatf_t q)
{
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return q;
}

static imu_quatf_t palm_runtime_quat_normalize(imu_quatf_t q)
{
  float norm = sqrtf((q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z));

  if (norm <= 0.0f) {
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    return q;
  }

  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;

  if (q.w < 0.0f) {
    q.w = -q.w;
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
  }

  return q;
}

static imu_quatf_t palm_runtime_apply_output_frame_remap(imu_quatf_t input)
{
#if (PALM_OUTPUT_FRAME_REMAP_ENABLE != 0U)
  imu_quatf_t remap;
  imu_quatf_t remap_conj;
  imu_quatf_t output;

  remap.w = PALM_OUTPUT_FRAME_REMAP_W;
  remap.x = PALM_OUTPUT_FRAME_REMAP_X;
  remap.y = PALM_OUTPUT_FRAME_REMAP_Y;
  remap.z = PALM_OUTPUT_FRAME_REMAP_Z;
  remap = palm_runtime_quat_normalize(remap);
  remap_conj = palm_runtime_quat_conjugate(remap);

  /* Frame transform: R' = C * R * C^T (quaternion sandwich form). */
  output = palm_runtime_quat_multiply(remap, input);
  output = palm_runtime_quat_multiply(output, remap_conj);
  return palm_runtime_quat_normalize(output);
#else
  return palm_runtime_quat_normalize(input);
#endif
}

static float palm_runtime_select_axis(const imu_vec3f_t *input, imu_axis_source_t source)
{
  switch (source) {
    case IMU_AXIS_SOURCE_X:
      return input->x;
    case IMU_AXIS_SOURCE_Y:
      return input->y;
    case IMU_AXIS_SOURCE_Z:
      return input->z;
    default:
      return 0.0f;
  }
}

static void palm_runtime_remap_vec3(const imu_vec3f_t *input,
                                    const imu_axis_remap_config_t *config,
                                    imu_vec3f_t *output)
{
  output->x = palm_runtime_select_axis(input, config->output_x_source);
  output->y = palm_runtime_select_axis(input, config->output_y_source);
  output->z = palm_runtime_select_axis(input, config->output_z_source);

  if (config->invert_x != 0U) {
    output->x = -output->x;
  }

  if (config->invert_y != 0U) {
    output->y = -output->y;
  }

  if (config->invert_z != 0U) {
    output->z = -output->z;
  }
}

static void palm_runtime_remap_sample(const imu_sample_t *sample,
                                      const imu_axis_remap_config_t *config,
                                      imu_sample_t *remapped_sample)
{
  *remapped_sample = *sample;
  palm_runtime_remap_vec3(&sample->accel_g, config, &remapped_sample->accel_g);
  palm_runtime_remap_vec3(&sample->gyro_dps, config, &remapped_sample->gyro_dps);
}

static void palm_runtime_add_sample(imu_sample_t *accumulator, const imu_sample_t *sample)
{
  accumulator->accel_g.x += sample->accel_g.x;
  accumulator->accel_g.y += sample->accel_g.y;
  accumulator->accel_g.z += sample->accel_g.z;
  accumulator->gyro_dps.x += sample->gyro_dps.x;
  accumulator->gyro_dps.y += sample->gyro_dps.y;
  accumulator->gyro_dps.z += sample->gyro_dps.z;
}

static void palm_runtime_scale_sample(imu_sample_t *sample, float scale)
{
  sample->accel_g.x *= scale;
  sample->accel_g.y *= scale;
  sample->accel_g.z *= scale;
  sample->gyro_dps.x *= scale;
  sample->gyro_dps.y *= scale;
  sample->gyro_dps.z *= scale;
}

static uint8_t palm_runtime_sample_is_still(const imu_sample_t *sample)
{
  float accel_norm;
  float gyro_norm;

  accel_norm = sqrtf((sample->accel_g.x * sample->accel_g.x) +
                     (sample->accel_g.y * sample->accel_g.y) +
                     (sample->accel_g.z * sample->accel_g.z));
  gyro_norm = sqrtf((sample->gyro_dps.x * sample->gyro_dps.x) +
                    (sample->gyro_dps.y * sample->gyro_dps.y) +
                    (sample->gyro_dps.z * sample->gyro_dps.z));

  if (fabsf(accel_norm - 1.0f) > PALM_ACCEL_NORM_STILL_TOL_G) {
    return 0U;
  }

  if (gyro_norm > PALM_GYRO_BIAS_MAX_STILL_DPS) {
    return 0U;
  }

  return 1U;
}

static void palm_runtime_accumulate_bias(palm_runtime_t *runtime, const imu_sample_t *sample)
{
  runtime->gyro_bias_accum_dps.x += sample->gyro_dps.x;
  runtime->gyro_bias_accum_dps.y += sample->gyro_dps.y;
  runtime->gyro_bias_accum_dps.z += sample->gyro_dps.z;
  runtime->calibration_samples++;
}

static void palm_runtime_finalize_bias(palm_runtime_t *runtime)
{
  if (runtime->calibration_samples > 0U) {
    float sample_scale = 1.0f / (float)runtime->calibration_samples;

    runtime->gyro_bias_dps.x = runtime->gyro_bias_accum_dps.x * sample_scale;
    runtime->gyro_bias_dps.y = runtime->gyro_bias_accum_dps.y * sample_scale;
    runtime->gyro_bias_dps.z = runtime->gyro_bias_accum_dps.z * sample_scale;
  } else {
    runtime->gyro_bias_dps.x = 0.0f;
    runtime->gyro_bias_dps.y = 0.0f;
    runtime->gyro_bias_dps.z = 0.0f;
  }

  runtime->calibration_complete = 1U;
  (void)mahony_filter_init(&runtime->mahony);
}

static void palm_runtime_subtract_bias(imu_sample_t *sample, const imu_vec3f_t *bias_dps)
{
  sample->gyro_dps.x -= bias_dps->x;
  sample->gyro_dps.y -= bias_dps->y;
  sample->gyro_dps.z -= bias_dps->z;
}

HAL_StatusTypeDef palm_runtime_init(palm_runtime_t *runtime, I2C_HandleTypeDef *bus)
{
  HAL_StatusTypeDef status;

  memset(runtime, 0, sizeof(*runtime));
  palm_runtime_reset_identity(&runtime->fused_attitude);
  palm_runtime_zero_reference_identity(&runtime->zero_reference);
  runtime->zero_pending = 0U;
  runtime->zero_captured = 0U;

  status = lsm6dsow_init(&runtime->imu_devices[0], bus, PALM_IMU0_I2C_ADDR_HAL);
  if (status != HAL_OK) {
    return status;
  }

  status = lsm6dsow_init(&runtime->imu_devices[1], bus, PALM_IMU1_I2C_ADDR_HAL);
  if (status != HAL_OK) {
    return status;
  }

  status = mahony_filter_init(&runtime->mahony);
  if (status != HAL_OK) {
    return status;
  }

  runtime->last_update_ms = HAL_GetTick();
  runtime->initialized = 1U;
  return HAL_OK;
}

void palm_runtime_process(palm_runtime_t *runtime)
{
  palm_protocol_payload_t payload;
  uint8_t frame[PALM_PROTOCOL_FRAME_SIZE];
  imu_sample_t samples[PALM_IMU_COUNT];
  imu_sample_t aligned_samples[PALM_IMU_COUNT];
  imu_sample_t fused_sample;
  uint32_t now_ms;
  uint32_t elapsed_ms;
  float delta_time_s;
  HAL_StatusTypeDef imu_status;
  HAL_StatusTypeDef filter_status;
  uint8_t active_imu_count = 0U;

  if (runtime->initialized == 0U) {
    return;
  }

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - runtime->last_update_ms;
  if (elapsed_ms < PALM_IMU_FILTER_PERIOD_MS) {
    return;
  }

  runtime->last_update_ms = now_ms;
  runtime->status = 0U;
  runtime->fused_attitude.valid = 0U;
  delta_time_s = (float)elapsed_ms / 1000.0f;
  if (delta_time_s <= 0.0f) {
    delta_time_s = PALM_IMU_FILTER_DT_S;
  }
  memset(samples, 0, sizeof(samples));
  memset(aligned_samples, 0, sizeof(aligned_samples));
  memset(&fused_sample, 0, sizeof(fused_sample));

  imu_status = lsm6dsow_read_sample(&runtime->imu_devices[0], &samples[0]);
  if (imu_status == HAL_OK) {
    runtime->status |= PALM_STATUS_IMU0_OK;
    palm_runtime_remap_sample(&samples[0],
                              &g_palm_imu_axis_remaps[0],
                              &aligned_samples[0]);
  }

  imu_status = lsm6dsow_read_sample(&runtime->imu_devices[1], &samples[1]);
  if (imu_status == HAL_OK) {
    runtime->status |= PALM_STATUS_IMU1_OK;
    palm_runtime_remap_sample(&samples[1],
                              &g_palm_imu_axis_remaps[1],
                              &aligned_samples[1]);
  }

  if ((runtime->status & PALM_STATUS_IMU0_OK) != 0U) {
    palm_runtime_add_sample(&fused_sample, &aligned_samples[0]);
    active_imu_count++;
  }

  if ((runtime->status & PALM_STATUS_IMU1_OK) != 0U) {
    palm_runtime_add_sample(&fused_sample, &aligned_samples[1]);
    active_imu_count++;
  }

  if (active_imu_count == 1U) {
    runtime->status |= PALM_STATUS_SINGLE_IMU_MODE;
  }

  if (active_imu_count > 0U) {
    palm_runtime_scale_sample(&fused_sample, 1.0f / (float)active_imu_count);
    fused_sample.timestamp_ms = now_ms;
  }

  if ((active_imu_count > 0U) && (runtime->calibration_complete == 0U)) {
    runtime->calibration_attempts++;

    if (palm_runtime_sample_is_still(&fused_sample) != 0U) {
      palm_runtime_accumulate_bias(runtime, &fused_sample);
    }

    if ((runtime->calibration_samples >= PALM_GYRO_BIAS_CALIBRATION_SAMPLES) ||
        (runtime->calibration_attempts >= PALM_GYRO_BIAS_CALIBRATION_TIMEOUT_SAMPLES)) {
      palm_runtime_finalize_bias(runtime);
    }
  }

  if (runtime->calibration_complete == 0U) {
    runtime->status |= PALM_STATUS_CALIBRATING;
  } else if (active_imu_count > 0U) {
    palm_runtime_subtract_bias(&fused_sample, &runtime->gyro_bias_dps);
    filter_status = mahony_filter_update(&runtime->mahony, &fused_sample, delta_time_s, &runtime->fused_attitude);
    if (filter_status == HAL_BUSY) {
      runtime->status |= PALM_STATUS_FILTER_WARMUP;
    } else if ((filter_status == HAL_OK) && (runtime->fused_attitude.valid != 0U)) {
      runtime->status |= PALM_STATUS_FUSION_READY;
    }
  }

  // ==============================
  // 核心修改：赋值 加速度 + 四元数
  // ==============================
  if (runtime->fused_attitude.valid != 0U) {

    imu_quatf_t palm_out = runtime->fused_attitude.quaternion;

    /* Apply fixed output-frame remap so USB quaternion matches host world
     * convention directly and host-side palm remap can be identity. */
    palm_out = palm_runtime_apply_output_frame_remap(palm_out);

    /* Palm-side Zero All rebasing: capture the current remapped output
     * quaternion on the first valid sample after a request, then emit
     * (inv(ref) * q_out) for subsequent frames. Only applied to the
     * USB-visible output; sensor fusion itself is untouched. */
    if (runtime->zero_pending != 0U) {
      runtime->zero_reference = palm_runtime_quat_normalize(palm_out);
      runtime->zero_captured = 1U;
      runtime->zero_pending = 0U;
    }

    if (runtime->zero_captured != 0U) {
      imu_quatf_t ref_conj = palm_runtime_quat_conjugate(runtime->zero_reference);
      palm_out = palm_runtime_quat_multiply(ref_conj, palm_out);
      palm_out = palm_runtime_quat_normalize(palm_out);
    }

    payload.orientation = palm_out;


    payload.accel_g = fused_sample.accel_g;

  } else {
    // 无效时：四元数 + 加速度都清零
    payload.orientation.w = 1.0f;
    payload.orientation.x = 0.0f;
    payload.orientation.y = 0.0f;
    payload.orientation.z = 0.0f;
    payload.accel_g.x = 0.0f;
    payload.accel_g.y = 0.0f;
    payload.accel_g.z = 0.0f;
  }

  // 固定信息不变
  payload.model_id = 0x94;
  payload.status = runtime->status;
  payload.node_id = PALM_NODE_ID;

  // 统一打包（包含加速度）
  palm_protocol_build_frame(&payload, frame);

  // USB 发送
  if (hUsbDeviceHS.pClassData == NULL) {
    runtime->usb_busy_count++;
    runtime->status |= PALM_STATUS_USB_BUSY;
    return;
  }

  if (CDC_Transmit_HS(frame, PALM_PROTOCOL_FRAME_SIZE) == USBD_OK) {
    runtime->frames_sent++;
  } else {
    runtime->usb_busy_count++;
    runtime->status |= PALM_STATUS_USB_BUSY;
  }
}


void palm_runtime_request_zero(palm_runtime_t *runtime)
{
  if (runtime == NULL) {
    return;
  }

  runtime->zero_captured = 0U;
  runtime->zero_pending = 1U;
}

void palm_runtime_clear_zero(palm_runtime_t *runtime)
{
  if (runtime == NULL) {
    return;
  }

  runtime->zero_pending = 0U;
  runtime->zero_captured = 0U;
  palm_runtime_zero_reference_identity(&runtime->zero_reference);
}
