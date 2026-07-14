#include "app/fingertip_runtime.h"

#include <math.h>
#include <string.h>

#include "imu/imu_config.h"
#include "protocol/palm_protocol.h"

// ===================== 配置宏：一键切换单/双IMU =====================
// 1 = 仅使用 IMU0（单IMU模式）
// 0 = 使用 IMU0 + IMU1（双IMU模式）
#define FINGERTIP_USE_SINGLE_IMU  0
// ======================================================================

static void fingertip_runtime_reset_identity(imu_attitude_t *attitude)
{
  memset(attitude, 0, sizeof(*attitude));
  attitude->quaternion.w = 1.0f;
}

static void fingertip_runtime_align_sample(const imu_sample_t *sample,
                                           float sign_x,
                                           float sign_y,
                                           float sign_z,
                                           imu_sample_t *aligned_sample)
{
  *aligned_sample = *sample;
  aligned_sample->accel_g.x *= sign_x;
  aligned_sample->accel_g.y *= sign_y;
  aligned_sample->accel_g.z *= sign_z;
  aligned_sample->gyro_dps.x *= sign_x;
  aligned_sample->gyro_dps.y *= sign_y;
  aligned_sample->gyro_dps.z *= sign_z;
}

#if !FINGERTIP_USE_SINGLE_IMU
// 双IMU模式专用函数，单IMU模式不编译
static void fingertip_runtime_add_sample(imu_sample_t *accumulator,
                                         const imu_sample_t *sample)
{
  accumulator->accel_g.x += sample->accel_g.x;
  accumulator->accel_g.y += sample->accel_g.y;
  accumulator->accel_g.z += sample->accel_g.z;
  accumulator->gyro_dps.x += sample->gyro_dps.x;
  accumulator->gyro_dps.y += sample->gyro_dps.y;
  accumulator->gyro_dps.z += sample->gyro_dps.z;
}

static void fingertip_runtime_scale_sample(imu_sample_t *sample, float scale)
{
  sample->accel_g.x *= scale;
  sample->accel_g.y *= scale;
  sample->accel_g.z *= scale;
  sample->gyro_dps.x *= scale;
  sample->gyro_dps.y *= scale;
  sample->gyro_dps.z *= scale;
}
#endif

static uint8_t fingertip_runtime_sample_is_still(const imu_sample_t *sample)
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

static void fingertip_runtime_accumulate_bias(fingertip_runtime_t *runtime,
                                              const imu_sample_t *sample)
{
  runtime->gyro_bias_accum_dps.x += sample->gyro_dps.x;
  runtime->gyro_bias_accum_dps.y += sample->gyro_dps.y;
  runtime->gyro_bias_accum_dps.z += sample->gyro_dps.z;
  runtime->calibration_samples++;
}

static void fingertip_runtime_finalize_bias(fingertip_runtime_t *runtime)
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

static void fingertip_runtime_subtract_bias(imu_sample_t *sample,
                                            const imu_vec3f_t *bias_dps)
{
  sample->gyro_dps.x -= bias_dps->x;
  sample->gyro_dps.y -= bias_dps->y;
  sample->gyro_dps.z -= bias_dps->z;
}

static HAL_StatusTypeDef fingertip_runtime_send_frame(fingertip_runtime_t *runtime,
                                                      const palm_protocol_payload_t *payload)
{
  if ((runtime == NULL) || (payload == NULL) || (runtime->tx_uart == NULL)) {
    return HAL_ERROR;
  }

  if (payload->node_id > PALM_MAX_LOCAL_NODE_INDEX) {
    return HAL_ERROR;
  }

  if (runtime->tx_busy != 0U) {
    runtime->frames_dropped++;
    runtime->transport_busy_latched = 1U;
    return HAL_BUSY;
  }

  palm_protocol_build_frame(payload, runtime->tx_frame);
  if (HAL_UART_Transmit_IT(runtime->tx_uart, runtime->tx_frame, PALM_PROTOCOL_FRAME_SIZE) != HAL_OK) {
    runtime->frames_dropped++;
    runtime->transport_busy_latched = 1U;
    return HAL_BUSY;
  }

  runtime->tx_busy = 1U;
  runtime->frames_sent++;
  return HAL_OK;
}

HAL_StatusTypeDef fingertip_runtime_init(fingertip_runtime_t *runtime,
                                         I2C_HandleTypeDef *imu_bus,
                                         UART_HandleTypeDef *tx_uart)
{
  HAL_StatusTypeDef status;

  if ((runtime == NULL) || (imu_bus == NULL) || (tx_uart == NULL)) {
    return HAL_ERROR;
  }

  memset(runtime, 0, sizeof(*runtime));
  runtime->imu_bus = imu_bus;
  runtime->tx_uart = tx_uart;
  fingertip_runtime_reset_identity(&runtime->fused_attitude);

  // 初始化 IMU0（必选）
  status = lsm6dsow_init(&runtime->imu_devices[0], imu_bus, PALM_IMU0_I2C_ADDR_HAL);
  if (status != HAL_OK) {
    return status;
  }

#if !FINGERTIP_USE_SINGLE_IMU
  // 双IMU模式：初始化 IMU1
  status = lsm6dsow_init(&runtime->imu_devices[1], imu_bus, PALM_IMU1_I2C_ADDR_HAL);
  if (status != HAL_OK) {
    return status;
  }
#endif

  status = mahony_filter_init(&runtime->mahony);
  if (status != HAL_OK) {
    return status;
  }

  runtime->last_update_ms = HAL_GetTick();
  runtime->initialized = 1U;
  return HAL_OK;
}

void fingertip_runtime_process(fingertip_runtime_t *runtime)
{
  palm_protocol_payload_t payload;

#if FINGERTIP_USE_SINGLE_IMU
  imu_sample_t sample;                // 单IMU：单个采样值
  imu_sample_t aligned_sample;        // 单IMU：轴对齐后数据
#else
  imu_sample_t samples[PALM_IMU_COUNT];
  imu_sample_t aligned_samples[PALM_IMU_COUNT];
  imu_sample_t fused_sample;
  uint8_t active_imu_count = 0U;
#endif
  uint32_t now_ms;
  uint32_t elapsed_ms;
  float delta_time_s;
  HAL_StatusTypeDef imu_status;
  HAL_StatusTypeDef filter_status;
  HAL_StatusTypeDef tx_status;

  if ((runtime == NULL) || (runtime->initialized == 0U)) {
    // 即使IMU未初始化，也要写入默认值
    extern SensorData system[MAX_BOARDS];
    system[0].quat_w = 1.0f;
    system[0].quat_x = 0.0f;
    system[0].quat_y = 0.0f;
    system[0].quat_z = 0.0f;
    system[0].accel_x = 0.0f;
    system[0].accel_y = 0.0f;
    system[0].accel_z = 0.0f;
    system[0].status = 0U;
    system[0].id = PALM_LOCAL_NODE_INDEX;
    system[0].model_id = 0x14;
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

#if FINGERTIP_USE_SINGLE_IMU
  // ===================== 单IMU模式逻辑 =====================
  memset(&sample, 0, sizeof(sample));
  memset(&aligned_sample, 0, sizeof(aligned_sample));

  // 仅读取 IMU0
  imu_status = lsm6dsow_read_sample(&runtime->imu_devices[0], &sample);
  if (imu_status == HAL_OK) {
    runtime->status |= PALM_STATUS_IMU0_OK;
    fingertip_runtime_align_sample(&sample,
                                   PALM_IMU0_AXIS_SIGN_X,
                                   PALM_IMU0_AXIS_SIGN_Y,
                                   PALM_IMU0_AXIS_SIGN_Z,
                                   &aligned_sample);
    aligned_sample.timestamp_ms = now_ms;
  }

  // ✅ 移除致命的return！即使IMU读取失败也要继续执行
#else
  // ===================== 双IMU模式逻辑 =====================
  memset(samples, 0, sizeof(samples));
  memset(aligned_samples, 0, sizeof(aligned_samples));
  memset(&fused_sample, 0, sizeof(fused_sample));

  // 读取 IMU0
  imu_status = lsm6dsow_read_sample(&runtime->imu_devices[0], &samples[0]);
  if (imu_status == HAL_OK) {
    runtime->status |= PALM_STATUS_IMU0_OK;
    fingertip_runtime_align_sample(&samples[0],
                                   PALM_IMU0_AXIS_SIGN_X,
                                   PALM_IMU0_AXIS_SIGN_Y,
                                   PALM_IMU0_AXIS_SIGN_Z,
                                   &aligned_samples[0]);
  }

  // 读取 IMU1
  imu_status = lsm6dsow_read_sample(&runtime->imu_devices[1], &samples[1]);
  if (imu_status == HAL_OK) {
    runtime->status |= PALM_STATUS_IMU1_OK;
    fingertip_runtime_align_sample(&samples[1],
                                   PALM_IMU1_AXIS_SIGN_X,
                                   PALM_IMU1_AXIS_SIGN_Y,
                                   PALM_IMU1_AXIS_SIGN_Z,
                                   &aligned_samples[1]);
  }

  // 双IMU数据融合
  if ((runtime->status & PALM_STATUS_IMU0_OK) != 0U) {
    fingertip_runtime_add_sample(&fused_sample, &aligned_samples[0]);
    active_imu_count++;
  }
  if ((runtime->status & PALM_STATUS_IMU1_OK) != 0U) {
    fingertip_runtime_add_sample(&fused_sample, &aligned_samples[1]);
    active_imu_count++;
  }

  if (active_imu_count == 1U) {
    runtime->status |= PALM_STATUS_SINGLE_IMU_MODE;
  }

  if (active_imu_count > 0U) {
    fingertip_runtime_scale_sample(&fused_sample, 1.0f / (float)active_imu_count);
    fused_sample.timestamp_ms = now_ms;
  }
#endif

  // ===================== 陀螺仪校准（通用逻辑） =====================
#if FINGERTIP_USE_SINGLE_IMU
  const imu_sample_t *calib_sample = &aligned_sample;
#else
  const imu_sample_t *calib_sample = &fused_sample;
#endif

  if (runtime->calibration_complete == 0U) {
    runtime->calibration_attempts++;
    if ((runtime->status & PALM_STATUS_IMU0_OK) != 0U &&
        fingertip_runtime_sample_is_still(calib_sample) != 0U) {
      fingertip_runtime_accumulate_bias(runtime, calib_sample);
    }
    if ((runtime->calibration_samples >= PALM_GYRO_BIAS_CALIBRATION_SAMPLES) ||
        (runtime->calibration_attempts >= PALM_GYRO_BIAS_CALIBRATION_TIMEOUT_SAMPLES)) {
      fingertip_runtime_finalize_bias(runtime);
    }
  }

  // ===================== 姿态融合（通用逻辑） =====================
  if (runtime->calibration_complete == 0U) {
    runtime->status |= PALM_STATUS_CALIBRATING;
  } else {
#if FINGERTIP_USE_SINGLE_IMU
    if ((runtime->status & PALM_STATUS_IMU0_OK) != 0U) {
      filter_status = mahony_filter_update(&runtime->mahony, &aligned_sample, delta_time_s, &runtime->fused_attitude);
    }
#else
    if (active_imu_count > 0U) {
      filter_status = mahony_filter_update(&runtime->mahony, &fused_sample, delta_time_s, &runtime->fused_attitude);
    }
#endif

    if (filter_status == HAL_BUSY) {
      runtime->status |= PALM_STATUS_FILTER_WARMUP;
    } else if ((filter_status == HAL_OK) && (runtime->fused_attitude.valid != 0U)) {
      runtime->status |= PALM_STATUS_FUSION_READY;
    }
  }

  // ===================== 数据发送（通用逻辑） =====================
  if (runtime->transport_busy_latched != 0U) {
    runtime->status |= PALM_STATUS_USB_BUSY;
  }

  // 引入全局仓库
  extern SensorData system[MAX_BOARDS];

  // ===================== 把 IMU 数据存入 system[0] =====================
  if (runtime->fused_attitude.valid != 0U) {
      system[0].quat_w = runtime->fused_attitude.quaternion.w;
      system[0].quat_x = runtime->fused_attitude.quaternion.x;
      system[0].quat_y = runtime->fused_attitude.quaternion.y;
      system[0].quat_z = runtime->fused_attitude.quaternion.z;

#if FINGERTIP_USE_SINGLE_IMU
      system[0].accel_x = aligned_sample.accel_g.x;
      system[0].accel_y = aligned_sample.accel_g.y;
      system[0].accel_z = aligned_sample.accel_g.z;
#else
      system[0].accel_x = fused_sample.accel_g.x;
      system[0].accel_y = fused_sample.accel_g.y;
      system[0].accel_z = fused_sample.accel_g.z;
#endif
  } else {
      // 无效时的默认安全值
      system[0].quat_w = 1.0f;
      system[0].quat_x = 0.0f;
      system[0].quat_y = 0.0f;
      system[0].quat_z = 0.0f;
      system[0].accel_x = 0.0f;
      system[0].accel_y = 0.0f;
      system[0].accel_z = 0.0f;
  }

  // 更新本地板子的状态和 ID
  system[0].status = runtime->status;
  system[0].id = PALM_LOCAL_NODE_INDEX; // 固定为 0
  system[0].model_id = 0x14;            // 你的硬件型号

  // 释放 IMU 的发送锁，因为现在发送归轮询管了
  runtime->transport_busy_latched = 0U;
}

void fingertip_runtime_on_uart_tx_complete(fingertip_runtime_t *runtime,
                                           UART_HandleTypeDef *huart)
{
  if ((runtime == NULL) || (huart == NULL) || (runtime->tx_uart == NULL)) {
    return;
  }

  if (huart == runtime->tx_uart) {
    runtime->tx_busy = 0U;
  }
}
